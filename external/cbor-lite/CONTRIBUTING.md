Contributing to CBOR-lite
=========================

Contributions to CBOR-lite are welcomed!

Coding guidelines
-----------------

* Use Modern C++.
* Use only standard language/library features available in C++ 14.
* Provide and run unit tests.
* Generally follow [Modern C++ Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md), except:
    - naming (follow conventions of existing code)
    - formatting (use 'reformat' build rule)
    - no GSL.
* Additional coding guidelines
    - avoid #ifdefs

Source Code Access and Branch Flow
----------------------------------

The primary source repository is hosted at [Bitbucket](https://bitbucket.org/isode/cbor-lite/) and can be checked out using

```
git clone https://bitbucket.org/isode/cbor-lite.git
```

'master' is the main *development* branch. New features, as well as bug fixes in not yet released features, should be developed against it, on a 'feature' branch.
'production' is the main *production* or *stable* branch. Bug fixes should generally be developed against it, on a 'bugfix' branch.


Submission guidelines
---------------------

To submit a patch, use:

```
git format-patch branch
```

where branch is the appropriate upstream branch (see Branch Flow discussion above). Then attach the patch to a new [issue report](https://bitbucket.org/isode/cbor-lite/issues).

Alternatively, submit a pull request.
