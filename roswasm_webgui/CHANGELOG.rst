^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roswasm_webgui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2022-04-05)
------------------
* Update roswasm_webgui-extras.cmake.in
* Merge pull request `#25 <https://github.com/nilsbore/roswasm_suite/issues/25>`_ from nilsbore/nilsbore-patch-1
  Update package.xml
* Update CMakeLists.txt
* Merge pull request `#2 <https://github.com/nilsbore/roswasm_suite/issues/2>`_ from nilsbore/master
  Update to new version
* Update package.xml
* Merge pull request `#20 <https://github.com/nilsbore/roswasm_suite/issues/20>`_ from Jollerprutt/stop_spam
  stop spaming :)
* Merge pull request `#19 <https://github.com/nilsbore/roswasm_suite/issues/19>`_ from Jollerprutt/add_get_states
  add get_states
* stop spaming :)
* add get_states
* Merge pull request `#18 <https://github.com/nilsbore/roswasm_suite/issues/18>`_ from nilsbore/set_bool
  Added special casing to handle std_srvs/SetBool
* Updated package versions
* Fixed bug in float slider
* Merge pull request `#17 <https://github.com/nilsbore/roswasm_suite/issues/17>`_ from nilsbore/python3-install
  Update CMakeLists.txt
* Update package.xml
* Merge pull request `#15 <https://github.com/nilsbore/roswasm_suite/issues/15>`_ from nilsbore/rename_library
  Renaming library for native
* Renaming library for native
* Merge pull request `#14 <https://github.com/nilsbore/roswasm_suite/issues/14>`_ from nilsbore/2.0
  API breaking 2.0 version compatible with subset of roscpp
* Updated package versions
* Added correct closing of program
* Fixed some bugs
* Fixed installed version
* Bugfix
* Added deps for native webgui, sdl2-image is not in rosdistro yet
* Webgui now runs on native as well
* Got example GUI working with new wasm api
* Merge pull request `#1 <https://github.com/nilsbore/roswasm_suite/issues/1>`_ from nilsbore/master
  Latest changes
* Merge remote-tracking branch 'origin/master' into transform_client
* Merge pull request `#11 <https://github.com/nilsbore/roswasm_suite/issues/11>`_ from Jollerprutt/master
  moved window sizing outside image and monlaunch widgets
* moved window sizing outside example widgets
* moved window sizing outside image and monlaunch
* Updated to work with new emscripten
* Update README.md
* Merge remote-tracking branch 'origin/master'
* Added example images for GUI
* Renamed the launch files
* Update README.md
* Added example startup script to roswasm_webgui
* Removed the old webserver
* Added monlaunch launch file
* Migrated to new launch file structure
* Update README.md
* Delete sam_webgui.launch
* Added rosmon_msgs dep
* Fixed building installing of imgui after move
* Moved imgui headers to make installing easier
* Removed the old sam library
* Got it compiling after moving
* Moved headers and src to different folders
* Added a separate monlaunch gui node
* Got rid of a lot of stuff through some magic
* Got everuthing to compile with catkin tools
* Added prebuilt versions of executables if emscripten is not available
* Added proper install targets
* Putting html files in bin location
* Changed to correct path
* Removed the old imgui files also
* Renamed webgui html
* Removed the old sam files
* Removed sam_msgs dep
* Add 'roswasm_webgui/' from commit '771d2f9aefbc98ae140862bac8b88479b4e579cf'
  git-subtree-dir: roswasm_webgui
  git-subtree-mainline: 10f1207f93206bd1a52148df3c2fb5016f0a524d
  git-subtree-split: 771d2f9aefbc98ae140862bac8b88479b4e579cf
* Contributors: Carl Ljung, Jollerprutt, Nils Bore
