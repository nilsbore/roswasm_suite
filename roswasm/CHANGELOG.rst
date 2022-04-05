^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roswasm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2022-04-05)
------------------
* Merge pull request `#25 <https://github.com/nilsbore/roswasm_suite/issues/25>`_ from nilsbore/nilsbore-patch-1
  Update package.xml
* Update package.xml
* Merge pull request `#2 <https://github.com/nilsbore/roswasm_suite/issues/2>`_ from nilsbore/master
  Update to new version
* Update package.xml
* Merge pull request `#21 <https://github.com/nilsbore/roswasm_suite/issues/21>`_ from Jollerprutt/comment_out_assert
  comment out assert(urlLength...
* comment out assert(urlLength...
* Merge pull request `#18 <https://github.com/nilsbore/roswasm_suite/issues/18>`_ from nilsbore/set_bool
  Added special casing to handle std_srvs/SetBool
* Updated package versions
* Updated roswasm to handle SetBool
* Fixed bug in float slider
* Merge pull request `#17 <https://github.com/nilsbore/roswasm_suite/issues/17>`_ from nilsbore/python3-install
  Update CMakeLists.txt
* Update package.xml
* Update CMakeLists.txt
* Update CMakeLists.txt
* Merge pull request `#16 <https://github.com/nilsbore/roswasm_suite/issues/16>`_ from nilsbore/new-ci
  Update CI
* Update run.py
* Update package.xml
* Update package.xml
* Merge pull request `#15 <https://github.com/nilsbore/roswasm_suite/issues/15>`_ from nilsbore/rename_library
  Renaming library for native
* Renaming library for native
* Merge pull request `#14 <https://github.com/nilsbore/roswasm_suite/issues/14>`_ from nilsbore/2.0
  API breaking 2.0 version compatible with subset of roscpp
* Updated package versions
* Fixed some bugs
* Added correct closing of program
* Fixed some bugs
* Fixed installed version
* Fixed looping glue
* Added some new methods to subscribers and timers
* Managed to get dependent packages running with native
* Added loop spin
* Added service clients that work for native and wasm
* Converted service clients to new convention
* Added the function template deduction problem on subscribe
* Got it to compile with native ros, not working with wasm right now
* Added init
* Fixed nodehandle init to mirror ros
* Added arguments to mimic native ros api
* Got the new structure running without pointers
* Merge pull request `#13 <https://github.com/nilsbore/roswasm_suite/issues/13>`_ from nilsbore/test_release
  Release building
* Update roswasm-extras.cmake.in
* Update CMakeLists.txt
* Merge pull request `#1 <https://github.com/nilsbore/roswasm_suite/issues/1>`_ from nilsbore/master
  Latest changes
* Merge pull request `#7 <https://github.com/nilsbore/roswasm_suite/issues/7>`_ from nilsbore/transform_client
  Added time and duration classes
* Merge remote-tracking branch 'origin/master' into transform_client
* Updated to work with new emscripten
* Updated to work with new emscripten
* Added time and duration classes
* Added new options to be able to run several nodes
* Fixed launch file install
* Changed to global namespace by default
* Added lanuch file that runs both web server and web socket
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Switched to python 2 for run script
* Should have been python3 version
* Some changes to maybe get tests to work with CI
* Added rostest for roswasm_tutorials
* Fixed building installed version of roswasm
* Merge remote-tracking branch 'origin/master'
* Delete LICENSE
* Removed a lot of unnecessary debug output
* Got rid of a lot of stuff through some magic
* Got everuthing to compile with catkin tools
* Added prebuilt versions of executables if emscripten is not available
* Added proper install targets
* Putting html files in bin location
* Removed the old sam files
* Added script for running stuff in devel or install space
* Add 'roswasm/' from commit 'a44690a68d5bd90fb417ebc17d82529f1220ecaf'
  git-subtree-dir: roswasm
  git-subtree-mainline: 8d1c68743cd9ba7e7e27b6a88c15f938beefd09f
  git-subtree-split: a44690a68d5bd90fb417ebc17d82529f1220ecaf
* Contributors: Carl Ljung, Jollerprutt, Nils Bore
