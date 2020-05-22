// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <Eigen/Core>
#include <iostream>

#include "TestUtility/UnitTest.h"

namespace open3d {
namespace unit_test {

void NotImplemented() {
    std::cout << "\033[0;32m"
              << "[          ] "
              << "\033[0;0m";
    std::cout << "\033[0;31m"
              << "Not implemented."
              << "\033[0;0m" << std::endl;

    GTEST_NONFATAL_FAILURE_("Not implemented");
}

void ExpectNear(const uint8_t* const v0,
                const uint8_t* const v1,
                const size_t& size) {
    for (size_t i = 0; i < size; i++) EXPECT_EQ(v0[i], v1[i]);
}

void ExpectNear(const std::vector<uint8_t>& v0,
                const std::vector<uint8_t>& v1) {
    EXPECT_EQ(v0.size(), v1.size());
    ExpectNear(v0.data(), v1.data(), v0.size());
}

void ExpectNear(const int* const v0, const int* const v1, const size_t& size) {
    for (size_t i = 0; i < size; i++) EXPECT_EQ(v0[i], v1[i]);
}

void ExpectNear(const std::vector<int>& v0, const std::vector<int>& v1) {
    EXPECT_EQ(v0.size(), v1.size());
    ExpectNear(v0.data(), v1.data(), v0.size());
}

void ExpectNear(const float* const v0,
                const float* const v1,
                const size_t& size,
                float threshold) {
    for (size_t i = 0; i < size; i++) {
        EXPECT_NEAR(v0[i], v1[i], threshold);
    }
}

void ExpectNear(const std::vector<float>& v0,
                const std::vector<float>& v1,
                float threshold) {
    EXPECT_EQ(v0.size(), v1.size());
    ExpectNear(v0.data(), v1.data(), v0.size(), threshold);
}

void ExpectNear(const double* const v0,
                const double* const v1,
                const size_t& size,
                double threshold) {
    for (size_t i = 0; i < size; i++) {
        EXPECT_NEAR(v0[i], v1[i], threshold);
    }
}

void ExpectNear(const std::vector<double>& v0,
                const std::vector<double>& v1,
                double threshold) {
    EXPECT_EQ(v0.size(), v1.size());
    ExpectNear(v0.data(), v1.data(), v0.size(), threshold);
}

}  // namespace unit_test
}  // namespace open3d
