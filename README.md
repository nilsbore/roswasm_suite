CBOR-lite
=========

CBOR-lite is C++ "header only" encoder/decoder implementation for the [Concise Binary Object Representation](https://cbor.io) (CBOR) available under a permissive [open source license](./COPYRIGHT.md). CBOR is specified in [RFC 7049](https://tools.ietf.org/html/rfc7049).

CBOR-lite is intended to support procedural encoding/decoding of application protocols using CBOR as their protocol data unit format.

CBOR-lite encoders are designed for applications requiring Canonical CBOR as described in [Section 3.9 of RFC 7049](https://tools.ietf.org/html/rfc7049#section-3.9).

The CBOR-lite API and its implementation is not yet mature. While it is believed to be mostly correct, it has not stood the test of time.

CBOR-lite is designed for use with Modern C++ ([C++14](https://en.wikipedia.org/wiki/C%2B%2B14) or later).  CBOR-lite has no external dependencies other than the C++ Standard Library.

The following aspects of CBOR are supported:

* Unsigned Integer (major type 0)
* Negative Integer (major type 1)
* Integer (major type 0 or 1 depending on signbit)
* Byte String (major type 2)
* Text String (major type 3) (without UTF-8 checking)
* Array of Data Items (major type 4)
* Map of Data Items (major type 5)
* Semantic Tagging (major type 6), namely Encoded CBOR data item (Tag 24)
* True/False (major type 7, value 20 and 21)
* Single- and Double-precision float point numbers (major type 7, value 26 and 27)

CBOR-lite supports rejection of non-minimal encodings of types 0 through 3. Other aspects of Canonical CBOR, as well as Strict Mode, are left to the application to implement.

The following aspects of CBOR are not supported:

* Half-precision float point numbers (major type 7, value types 24)
* Indefinite-length encodings

CBOR-lite is not intended to be a generic decoder as described in [Section 3.2 of RFC 7049](https://tools.ietf.org/html/rfc7049#section-3.2).

CBOR-lite is open-source. Community contributions are welcomed. See [contributing guidelines](CONTRIBUTING.md) for more information.

More information about CBOR-lite is available at [https://bitbucket.org/isode/cbor-lite/wiki/Home](https://bitbucket.org/isode/cbor-lite/wiki/Home).

See [COPYRIGHT.md](./COPYRIGHT.md) for [copyright and other legal notices](./COPYRIGHT.md).
