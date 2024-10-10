
#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {

        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();

    }

    /* bool CollisionDetector::inFreeSpace(const double* world_pos, double safety_radius) {

        // Convert world coordinates to map coordinates
        unsigned int mx, my;
        if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)) {
            // The position is outside the costmap boundaries
            return false;
        }

        // Define a threshold for what is considered "free space"
        unsigned char cost = costmap_->getCost(mx, my);

        // Check if the current cell is an obstacle or lethal zone
        if (cost >= costmap_2d::LETHAL_OBSTACLE) {
            return false;
        }

        // If a safety radius is defined, check the neighboring cells within that radius
        unsigned int cell_radius = static_cast<unsigned int>(std::ceil(safety_radius / resolution_));
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
                unsigned int neighbor_mx = mx + dx;
                unsigned int neighbor_my = my + dy;

                // Ensure that the neighbor coordinates are within the map bounds
                if (neighbor_mx < costmap_->getSizeInCellsX() && neighbor_my < costmap_->getSizeInCellsY()) {
                    unsigned char neighbor_cost = costmap_->getCost(neighbor_mx, neighbor_my);

                    // If any cell within the radius is an obstacle, return false
                    if (neighbor_cost >= costmap_2d::LETHAL_OBSTACLE) {
                        return false;
                    }
                }
            }
        }

        // If no obstacles are found within the radius, it's considered free space
        return true;
    } */


   bool CollisionDetector::inFreeSpace(const double* world_pos) {
    // Convert world coordinates to map coordinates
    unsigned int mx, my;
    if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)) {
        return false;  // The position is outside the costmap boundaries
    }

    // Get the cost at the map coordinates
    unsigned char cost = costmap_->getCost(mx, my);

    // Add a buffer to account for safe distance from obstacles
    unsigned char buffer_threshold = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // Inscribed radius
    
    // If cost is within dangerous levels or near obstacle buffer
    if (cost >= buffer_threshold) {
        return false;  // It's not free space (too close to obstacles)
    }

    return true;  // It's free space
}




    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {

        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return ( !inFreeSpace(point_b) ) ? true : false;

        } else {
            
            int num_steps = static_cast<int>(floor(dist/resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {

                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }

    }

};
