#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      double px = (double)x / (num_width_points - 1) * width;
      double py = (double)y / (num_height_points - 1) * height;
      Vector3D position;

      if (orientation == HORIZONTAL) {
        position = Vector3D(px, 1.0, py);
      } else { // VERTICAL
        double random_offset = ((double)rand() / RAND_MAX) * 0.002 - 0.001;
        position = Vector3D(px, py, random_offset);
      }

      bool is_pinned = false;
      for (const auto &p : pinned) {
        // std::cout << "Checking pinned position: " << p[0] << ", " << p[1] << std::endl;
        if (p[0] == x && p[1] == y) {
          is_pinned = true;
          break;
        }
      }

      point_masses.emplace_back(position, is_pinned);
    }
  }

  // create springs
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      int idx = y * num_width_points + x;

      // Structural (horizontal and vertical)
      if (x > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - 1], STRUCTURAL);
      }
      if (y > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - num_width_points], STRUCTURAL);
      }

      // Shearing (diagonal springs)
      if (x > 0 && y > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - num_width_points - 1], SHEARING);
      }
      if (x < num_width_points - 1 && y > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - num_width_points + 1], SHEARING);
      }

      // Bending (horizontal and vertical) * 2
      if (x > 1) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - 2], BENDING);
      }
      if (y > 1) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - 2 * num_width_points], BENDING);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // reset forces
  for (auto &pm : point_masses) {
    pm.forces = Vector3D();
  }

  // compute all external accelerations and apply
  Vector3D total_external_force;
  for (const auto &acceleration : external_accelerations) {
    total_external_force += mass * acceleration;
  }
  for (auto &pm : point_masses) {
    pm.forces += total_external_force;
  }

  // compute spring forces
  for (const auto &spring : springs) {
    // skip if disabled
    if ((spring.spring_type == STRUCTURAL && !cp->enable_structural_constraints) ||
        (spring.spring_type == SHEARING && !cp->enable_shearing_constraints) ||
        (spring.spring_type == BENDING && !cp->enable_bending_constraints)) {
      continue;
    }

    Vector3D dir = spring.pm_b->position - spring.pm_a->position;
    double length = dir.norm();
    double ks = cp->ks;
    if (spring.spring_type == BENDING) {
      ks *= 0.2;
    }
    Vector3D force = ks * (length - spring.rest_length) * dir.unit();

    spring.pm_a->forces += force;
    spring.pm_b->forces -= force;
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (auto &pm : point_masses) {
    if (!pm.pinned) {
      Vector3D acceleration = pm.forces / mass;
      Vector3D position = pm.position + (1 - cp->damping / 100.0) * (pm.position - pm.last_position) + acceleration * pow(delta_t, 2);
      pm.last_position = pm.position;
      pm.position = position;
    }
  }

  // Handle self-collisions.
  build_spatial_map();
  for (auto &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }

  // Handle collisions with other primitives
  for (auto &pm : point_masses) {
    for (auto &obj : *collision_objects) {
      obj->collide(pm);
    }
  }

  // Spring length constraint
  for (auto &spring : springs) {
    Vector3D dir = spring.pm_b->position - spring.pm_a->position;
    double curr_l = dir.norm();
    double max_l = spring.rest_length * 1.1;

    if (curr_l > max_l) {
      Vector3D correction = dir.unit() * (curr_l - max_l);
      if (!spring.pm_a->pinned && !spring.pm_b->pinned) {
        spring.pm_a->position += correction * 0.5;
        spring.pm_b->position -= correction * 0.5;
      } else if (!spring.pm_a->pinned) {
        spring.pm_a->position += correction;
      } else if (!spring.pm_b->pinned) {
        spring.pm_b->position -= correction;
      }
    }
  }
}


void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (auto &pm : point_masses) {
    float hash = hash_position(pm.position);
    if (map.find(hash) == map.end()) {
      map[hash] = new vector<PointMass *>();
    }
    // std::cout << "Adding point mass at hash: " << hash << std::endl;
    map[hash]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  float hash = hash_position(pm.position);

  Vector3D correction;
  int count = 0;

  for (PointMass *p : *map[hash]) {
    if (p == &pm) continue;

    Vector3D dir = pm.position - p->position;
    double distance = dir.norm();

    if (distance < 2 * thickness) {
      Vector3D correction_vector = dir.unit() * (2 * thickness - distance);
      correction += correction_vector;
      count++;
    }
  }

  if (count > 0) {
    correction /= count;
    correction /= simulation_steps;
    pm.position += correction;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3 * width / num_width_points;
  double h = 3 * height / num_height_points;
  double t = max(w, h);

  int x = floor(pos.x / w);
  int y = floor(pos.y / h);
  int z = floor(pos.z / t);

  // random large primes to reduce collisions
  return (float)((x * 73856093) ^ (y * 19349669) ^ (z * 83492803)) / 1e9;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
