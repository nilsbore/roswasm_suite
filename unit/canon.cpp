// This file is part of CBOR-lite which is copyright Isode Limited
// and others and released under a MIT license. For details, see the
// COPYRIGHT.md file in the top-level folder of the CBOR-lite software
// distribution.
#include "../include/cbor-lite/codec.h"
#include <boost/test/unit_test.hpp>
#include <tuple>

BOOST_AUTO_TEST_SUITE(cbor_lite)
BOOST_AUTO_TEST_SUITE(canon)

BOOST_AUTO_TEST_CASE(toolong) {
    std::int_fast64_t small = 23u;
    const std::vector<std::string> cases{
        "\x18\x17",
        std::string("\x19\x00\x17", 3),
        std::string("\x1a\x00\x00\00\x17", 5),
        std::string("\x1b\x00\x00\x00\x00\x00\x00\x00\x17", 9),
    };
    for (const auto& encoding : cases) {
        BOOST_CHECK_NO_THROW({
            std::uint_fast64_t value = 0u;
            auto pos = std::begin(encoding);
            auto len = CborLite::decodeUnsigned(pos, std::end(encoding), value);
            BOOST_CHECK(pos == std::end(encoding));
            BOOST_CHECK_EQUAL(len, encoding.size());
            BOOST_CHECK_EQUAL(value, small);
            pos = std::begin(encoding);
            BOOST_CHECK_EXCEPTION(CborLite::decodeUnsigned(pos, std::end(encoding), value, CborLite::Flag::requireMinimalEncoding),
                CborLite::Exception, [](CborLite::Exception const& e) {
                    const std::string expected = "CBOR Exception: encoding not minimal";
                    BOOST_CHECK_EQUAL(e.what(), expected);
                    return e.what() == expected;
                });
        });
    }
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()
