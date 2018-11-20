^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phoxi_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.5
-----------------
* GetDeviceList.srv - message and success response parameters added, len response parameter will be removed in next release.
* ConnectCamera.srv added message and success response parameters.
* Bool.srv added to replace IsAcquiring.srv and IsConnected.srv in next release.
* Empty.srv added to replace std_srvs/Empty in next release.
* TriggerImage.srv added message response parameter.
* GetFrame.srv added message response parameter.
* SaveFrame.srv added message response parameter.
* GetHardwareIdentification.srv added message success response parameter.
* GetSuportedCapturingModes.srv added message success response parameter.
* Added V2/is_connected, V2/is_acquiring, V2/start_acquisition, V2/stop_acquisition services.
* trigger_image service publish frame to Ros topics automatically
* get_frame service will take new frane on negative number input and publish it, on invalid frame number success will be false and message filled with error
* V2/set_transformation and V2/set_coordination_space services added
* TriggerImage.srv id parameter added
* PhoXi 3D Scanner urdf
* Diagnostic messages added

1.1.4 (2016-10-24)
------------------
* Update README.md
* Merge remote-tracking branch 'origin/master'
* add launch file
* Update README.md
* fix
* fix cmake_minimum_version
  fix package.xml
* Update README.md
* Update README.md
* Update README.md
* disable opencv, add pcl to package.xml
* fix bug
* Merge remote-tracking branch 'origin/master'
* fix normal_map
* change pointcloud values from mm to meters
* Update README.md
  add Generate package to README
* Contributors: Matej Sladek, jzizka

1.1.3 (2016-08-23)
------------------

0.0.3 (2016-08-16)
------------------
* fix bug
* Contributors: Matej Sladek

0.0.2 (2016-08-15)
------------------
* add gitignore
* Contributors: Matej Sladek

0.0.1 (2016-08-15)
------------------
* change path to phoxi, remove boost in cmake, rename confidence parameter, remove acquisition_time parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#1 <https://github.com/photoneo/phoxi_camera/issues/1>`_ from photoneo/matej
  Services, example
* clean
* add example
* clean project and add isConnected service
* fix services
* add services
* add services
* fix services, delete AddTwoInts, add GetdeviceList
* add example service
* add parameters
* change paths to PhoXiControl libs and include dir
* init changes
* init
* Initial commit
* Contributors: Matej Sladek
