#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <iostream>

TEST_SUITE("Smoke_tslam_tests")
{
    TEST_CASE("Smoke_tslam_test")
    {
        // call the runtime from here
        // TODO: put everything needed for tests in src/tests/data
        std::cout << "Hello world!" << std::endl;
        CHECK(true);
    }

}