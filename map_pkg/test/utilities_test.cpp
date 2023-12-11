#include <gtest/gtest.h>

#include <map_pkg/utilities.hpp>

TEST(map_pkg, obstacle_overlap_cylinders)
{
    obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};
    obstacle obs2 {1.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};

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
    obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, obstacle_type::BOX};
    obstacle obs2 {0.0, 0.0, 0.0, 2.0, 2.0, obstacle_type::BOX};

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
    obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};
    obstacle obs2 {0.0, 0.0, 0.0, 2.0, 2.0, obstacle_type::BOX};

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
    std::vector<obstacle> obss1 = {
        obstacle {2.086390, -1.625925, 1.845027, 0.0, 0.0, obstacle_type::CYLINDER},
        obstacle {0.0, 2.506032, 1.445556, 1.635741, 2.334744, obstacle_type::BOX},
        obstacle {2.178377, 3.579431, -2.871252, 0.0, 0.0, obstacle_type::CYLINDER},
        obstacle {1.544447, -2.109923, -2.835579, 0.0, 0.0, obstacle_type::CYLINDER},
        obstacle {1.905409, -0.093301, -7.651071, 0.0, 0.0, obstacle_type::CYLINDER}
    };

    std::vector<obstacle> obss2 = {};

    for (auto obs : obss1){
        EXPECT_FALSE(overlaps(obs, obss2));
        obss2.push_back(obs);
        std::cout << "Added obstacle " << obs.x << ", " << obs.y << std::endl;
        EXPECT_TRUE(overlaps(obs, obss2));
    }
}


TEST(map_pkg, obstacle_cylinder_in_map_square)
{
    obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};
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
    obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};
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
    obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, obstacle_type::BOX};
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
    obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, obstacle_type::BOX};
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
    obstacle obs1 {1.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};
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
    obstacle obs1 {0.0, 0.0, 0.0, 2.0, 2.0, obstacle_type::BOX};
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



int main (int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}