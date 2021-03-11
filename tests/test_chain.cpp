#define CATCH_CONFIG_MAIN
#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

#include "catch.hpp"

#include "./../chain/include/chain.h"

TEMPLATE_TEST_CASE("A 3 joint chain can be manipulated", "[joint-chain]",
                   float) {
    GIVEN("A 3 joint chain") {
        using vector3_type = Eigen::Matrix<TestType, 3, 1>;
        using quat_type = Eigen::Quaternion<TestType>;

        auto joint_chain = chain::JointChain<TestType>();

        auto& j0 = joint_chain.base();

        // oriented in z axis
        TestType pos1[3] = {1, 0, 0};
        TestType ori1[4] = {0, -.707, 0, .707};  // x, y, z, w
        auto& j1 = joint_chain.add_joint(pos1, ori1);

        // oriented in y axis
        TestType pos2[3] = {1, 1, 0};
        TestType ori2[4] = {0, 0, .707, .707};  // x, y, z, w
        auto& j2 = joint_chain.add_joint(pos2, ori2);

        WHEN("The base is rotated 90 degrees") {
            j0.set_angle(M_PI / 2);

            REQUIRE(j0.get_angle() == (TestType)(M_PI / 2));

            THEN("the locations of the other joints update appropriately") {
                REQUIRE(j1.get_position().isApprox(vector3_type(1, 0, 0)));
                REQUIRE(j1.get_orientation().isApprox(
                        quat_type(.5, .5, -.5, -.5)));
                REQUIRE(j2.get_position().isApprox(vector3_type(1, 0, 1)));
                REQUIRE(j2.get_orientation().isApprox(
                        quat_type(.5, .5, -.5, .5)));
            }

            AND_WHEN("we then rotate the first joint by -90 degrees") {
                std::cout << "********\n";

                j1.set_angle(M_PI / 2);

                REQUIRE(j1.get_angle() == (TestType)(M_PI / 2));

                THEN("the arm will be straight") {
                    REQUIRE(j1.get_position().isApprox(vector3_type(1, 0, 0)));
                    REQUIRE(j1.get_orientation().isApprox(
                            quat_type(.5, .5, -.5, -.5)));
                    REQUIRE(j2.get_position().isApprox(vector3_type(1, 0,
                    1)));
                    REQUIRE(j2.get_orientation().isApprox(quat_type(.707,
                    .707, 0, 0)));
                }
            }
        }
    }
}
