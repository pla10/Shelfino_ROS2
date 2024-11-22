#include <gtest/gtest.h>

#include <map_pkg/utilities.hpp>

TEST(map_pkg, obstacle_overlap_cylinders)
{
    Obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};
    Obstacle obs2 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};

    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 2.0;
    obs2.y = 0.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 0.0;
    obs2.y = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 0.5;
    obs2.y = 0.5;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 1.0;
    obs2.y = 1.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 1.5;
    obs2.y = 1.5;
    EXPECT_FALSE(overlaps(obs1, obs2));

    obs2.x = 0.0;
    obs2.y = 0.0;
    obs1.x = 0.0;
    obs1.x = 0.0;
    obs1.radius = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs1.radius = 1.0;
    obs2.radius = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));


    obs2.x = 0.0;
    obs2.y = 0.0;
    obs1.x = 1.0;
    obs1.y = 1.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 3.0;
    obs2.y = 3.0;
    obs2.radius = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));
}

TEST(map_pkg, obstacle_overlap_rectangles)
{
    Obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, 0.0, OBSTACLE_TYPE::BOX};
    Obstacle obs2 {0.0, 0.0, 0.0, 2.0, 2.0, 0.0, OBSTACLE_TYPE::BOX};

    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 2.0;
    obs2.y = 0.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 0.0;
    obs2.y = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 0.5;
    obs2.y = 0.5;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 1.0;
    obs2.y = 1.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 2.5;
    obs2.y = 2.5;
    EXPECT_FALSE(overlaps(obs1, obs2));

    obs2.x = 0.0;
    obs2.y = 0.0;
    obs1.x = 1.0;
    obs1.y = 1.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 3.0;
    obs2.y = 3.0;
    obs2.dx = 2.0;
    obs2.dy = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));
}

TEST(map_pkg, obstacle_overlap_rectangles_cylinder)
{
    Obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};
    Obstacle obs2 {0.0, 0.0, 0.0, 2.0, 2.0, 0.0, OBSTACLE_TYPE::BOX};

    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 2.0;
    obs2.y = 0.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 0.0;
    obs2.y = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 0.5;
    obs2.y = 0.5;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 1.0;
    obs2.y = 1.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 2.5;
    obs2.y = 2.5;
    EXPECT_FALSE(overlaps(obs1, obs2));

    obs2.x = 0.0;
    obs2.y = 0.0;
    obs1.x = 1.0;
    obs1.y = 1.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs2.x = 3.0;
    obs2.y = 3.0;
    obs2.dx = 2.0;
    obs2.dy = 2.0;
    EXPECT_FALSE(overlaps(obs1, obs2));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.radius = 10.0;
    obs2.x = 0.0;
    obs2.y = 0.0;
    obs2.dx = 2.0;
    obs2.dy = 2.0;
    EXPECT_TRUE(overlaps(obs1, obs2));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.radius = 10.0;
    obs2.x = 0.0;
    obs2.y = 0.0;
    obs2.dx = 20.0;
    obs2.dy = 20.0;
    EXPECT_TRUE(overlaps(obs1, obs2));
}

TEST(map_pkg, overlaps_obstacles){
    std::vector<Obstacle> obss1 = {
        Obstacle {2.086390, -1.625925, 1.845027, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER},
        Obstacle {0.0, 2.506032, 1.445556, 1.635741, 2.334744, 0.0, OBSTACLE_TYPE::BOX},
        Obstacle {2.178377, 3.579431, -2.871252, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER},
        Obstacle {1.544447, -2.109923, -2.835579, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER},
        Obstacle {1.905409, -0.093301, -7.651071, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER}
    };

    std::vector<Obstacle> obss2 = {};

    for (auto obs : obss1){
        EXPECT_FALSE(overlaps(obs, obss2));
        obss2.push_back(obs);
        EXPECT_TRUE(overlaps(obs, obss2));
    }
}


TEST(map_pkg, obstacle_cylinder_in_map_square)
{
    Obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};
    std::string map = "rectangle";
    double dx = 10;
    double dy = 10;

    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = -5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = -5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 4.0;
    obs1.y = 0.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.radius = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 100.0;
    obs1.y = 100.0;
    obs1.radius = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));
}

TEST(map_pkg, obstacle_cylinder_in_map_rectangle)
{
    Obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};
    std::string map = "rectangle";
    double dx = 10;
    double dy = 20;

    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = -10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = -10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.radius = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 100.0;
    obs1.y = 100.0;
    obs1.radius = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));
}

TEST(map_pkg, obstacle_square_in_map_square)
{
    Obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, 0.0, OBSTACLE_TYPE::BOX};
    std::string map = "rectangle";
    double dx = 10;
    double dy = 10;

    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = -5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = -5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 4.0;
    obs1.y = 0.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.dx = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.dx = 2.0;
    obs1.dy = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 100.0;
    obs1.y = 100.0;
    obs1.dx = 2.0;
    obs1.dy = 2.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));
}

TEST(map_pkg, obstacle_square_in_map_rectangle)
{
    Obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, 0.0, OBSTACLE_TYPE::BOX};
    std::string map = "rectangle";
    double dx = 10;
    double dy = 20;

    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 5.0;
    obs1.y = -10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -5.0;
    obs1.y = -10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.dx = 10.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.dx = 2.0;
    obs1.dy = 20.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 100.0;
    obs1.y = 100.0;
    obs1.dx = 2.0;
    obs1.dy = 2.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));
}

TEST(map_pkg, obstacle_cylinder_in_map_hexagon)
{
    Obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};
    std::string map = "hexagon";
    double dx = 6;
    double dy = dx;

    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -4.0;
    obs1.y = 0.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 4.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = -4.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 4.0;
    obs1.y = 0.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = dx;
    obs1.y = 0.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = dx;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.radius = sqrt(3)*dx/2.0-0.0001;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 100.0;
    obs1.y = 100.0;
    obs1.radius = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));
}

TEST(map_pkg, obstacle_rect_in_map_hexagon)
{
    Obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, 0.0, OBSTACLE_TYPE::BOX};
    std::string map = "hexagon";
    double dx = 6;
    double dy = dx;

    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = -4.0;
    obs1.y = 0.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 4.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = -4.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 4.0;
    obs1.y = 0.0;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = dx;
    obs1.y = 0.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = dx;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 0.0;
    obs1.y = 0.0;
    obs1.dx = dx;
    obs1.dy = dx;
    EXPECT_TRUE(is_inside_map(obs1, map, dx, dy));

    obs1.x = 100.0;
    obs1.y = 100.0;
    obs1.radius = 5.0;
    EXPECT_FALSE(is_inside_map(obs1, map, dx, dy));
}

TEST(map_pkg, rotate_obstacle_overlap){
    Obstacle obs1 {0.0, 0.5, 0.0, 1.0, 1.0, 0.0, OBSTACLE_TYPE::BOX};
    Obstacle obs2 {0.0, 0.0, 0.0, 1.0, 1.0, 0.0, OBSTACLE_TYPE::BOX};

    for (int i=0; i<360; i++){
        EXPECT_TRUE(overlaps(obs1, obs2));
        obs1.yaw = M_PI/360.0*i;
    }

    obs1.yaw = 0.0;
    obs2.x = 20.0;

    for (int i=0; i<360; i++){
        EXPECT_FALSE(overlaps(obs1, obs2));
        obs1.yaw = M_PI/360.0*i;
    }

    obs1.yaw = 0.0;
    obs1.x = 0.0;
    obs2.x = 1;

    double v_x = obs1.x + obs1.dx/2.0;
    double v_y = obs1.y + obs1.dy/2.0;

    for (int i=0; i<90; i++){
        // Rotation is counterclockwise, so we need to invert the sign
        double new_v_x = v_x*cos(-obs1.yaw) - v_y*sin(-obs1.yaw);

        if (new_v_x < obs2.x-obs2.dx/2.0){
            EXPECT_FALSE(overlaps(obs1, obs2));
        }
        else{
            EXPECT_TRUE(overlaps(obs1, obs2));
        }
        obs1.yaw += 2.0*M_PI/360;
    }
}

TEST(map_pkg, random_obstacle_test){
    // [send_obstacles-9] 	Obstacle: Cylinder: x: 3.69466 y: -3.6409 radius: 0.871469
    // [send_obstacles-9] 	Obstacle: Box: x: 4.3999 y: -4.30706 dx: 1 dy: 1 yaw: 3.14159
    Obstacle obs1 {0.0, 4.3999, -4.30706, 1, 1, M_PI, OBSTACLE_TYPE::BOX};
    Obstacle obs2 {0.871469, 3.69466, -3.6409, 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};

    EXPECT_TRUE(overlaps(obs1, obs2));
}

TEST(map_pkg, random_in_map_test){
    Obstacle obs {0.0, -0.733781, 5.177172, 0.617572, 0.855495, 0.0, OBSTACLE_TYPE::BOX};
    
    std::string map = "rectangle";
    double dx = 10;
    double dy = 10;

    EXPECT_FALSE(is_inside_map(obs, map, dx, dy));
}


int main (int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    testing::GTEST_FLAG(filter) = "map_pkg.random_obstacle_test";
    return RUN_ALL_TESTS();
}