// This file is part of CBOR-lite which is copyright Isode Limited
// and others and released under a MIT license. For details, see the
// COPYRIGHT.md file in the top-level folder of the CBOR-lite software
// distribution.
#include "../include/cbor-lite/codec.h"
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(cbor_lite)
BOOST_AUTO_TEST_SUITE(basic)

BOOST_AUTO_TEST_CASE(length) {
    const std::vector<std::pair<std::size_t, std::size_t>> cases{
        {0, 0},
        {1, 0},
        {23, 0},
        {24, 1},
        {255, 1},
        {256, 2},
        {65535, 2},
        {65536, 4},
        {4294967295, 4},
        {4294967296, 8},
    };
    for (const auto& test : cases) {
        BOOST_CHECK_NO_THROW({
            auto len = CborLite::length(test.first);
            BOOST_TEST(len == test.second, "CBOR length for " << test.first << " is " << len << " not " << test.second);
        });
    }
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()
