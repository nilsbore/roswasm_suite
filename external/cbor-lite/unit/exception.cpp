// This file is part of CBOR-lite which is copyright Isode Limited
// and others and released under a MIT license. For details, see the
// COPYRIGHT.md file in the top-level folder of the CBOR-lite software
// distribution.
#include "../include/cbor-lite/codec.h"
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(cbor_lite)
BOOST_AUTO_TEST_SUITE(exception)

BOOST_AUTO_TEST_CASE(basic) {
    BOOST_CHECK_EXCEPTION(throw CborLite::Exception(), CborLite::Exception,
        [](CborLite::Exception const& e) { return e.what() == std::string("CBOR Exception"); });

    BOOST_CHECK_EXCEPTION(throw CborLite::Exception("cstring"), CborLite::Exception,
        [](CborLite::Exception const& e) { return e.what() == std::string("CBOR Exception: cstring"); });

    BOOST_CHECK_EXCEPTION(throw CborLite::Exception(std::string("string")), CborLite::Exception,
        [](CborLite::Exception const& e) { return e.what() == std::string("CBOR Exception: string"); });

    CborLite::Exception ex("another string");
    BOOST_CHECK_EXCEPTION(throw CborLite::Exception(ex), CborLite::Exception,
        [](CborLite::Exception const& caught) { return caught.what() == std::string("CBOR Exception: another string"); });
    BOOST_CHECK_EQUAL(ex.what(), std::string("CBOR Exception: another string"));
    BOOST_CHECK_EXCEPTION(throw CborLite::Exception(std::move(ex)), CborLite::Exception,
        [](CborLite::Exception const& caught) { return caught.what() == std::string("CBOR Exception: another string"); });
    BOOST_CHECK_EQUAL(ex.what(), std::string("")); // note the odd state, CborLite::Exception not expected to be reused after move
}

BOOST_AUTO_TEST_CASE(inheritance) {
    BOOST_CHECK_NO_THROW(try { throw CborLite::Exception(); } catch (
        const std::exception& e) { BOOST_CHECK_EQUAL(e.what(), std::string("CBOR Exception")); });
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()
