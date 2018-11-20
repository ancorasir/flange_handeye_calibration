# Test interfaces for phoxi_camera node
## Requirements
* Installed PhoXi Control
* Installed pytest: sudo apt-get install python-pytest

## How to run tests
Before run of tests set scanner ID or file camera ID in the **interfaces/config.py**, 
it is needed for connect to camera. Default camera ID for testing is "InstalledExamples-PhoXi-example".

Compile tests: 
```bash
catkin_make -DCATKIN_ENABLE_TESTING=TRUE
```

If you have compiled tests, you can only run them via rostest:
```bash
rostest phoxi_camera test_phoxi_interface_class.test            # unittest for PhoXiInterface class
rostest phoxi_camera test_phoxi_camera_interfaces_exist.test    # this test check if all interfaces exist
rostest phoxi_camera test_phoxi_camera_ros_interfaces.test      # this test interact with phoxi_camera node and test it 
```

Find more options of rostest:
```bash
rostest -h
```

## Output of test
After run of a test, base information from the test are written to console.
For addition information check files:

Test result:
```bash
~/.ros/test_results/phoxi_camera/rostest-tests_phoxi_camera_interfaces.xml
```

More detailed test log:
```bash
~/.ros/log/rostest-X-Y.log
```

## How it function
These tests are realized via launch files with extension .test.
In this special launch file is included tested launch file which runs
target ros node and load parameters. The special launch file also launch
testing node which consist of python unittests, this node interact with
tested node and perform tests.
