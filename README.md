# An example ROS 2 C++ project

## How to create a similar project

This section list the steps that were taken to generate this repository.

They provide an outline of the creation process which may be reproduced to
create other similar ROS 2 projects.

- Load ROS 2, e.g. Foxy binary installation:

  ```sh
  source /opt/ros/foxy/setup.bash
  ```

- Create a root directory for the new repository, enter it, and
  initialize a `git` repo:

  ```sh
  mkdir hello-perftest
  cd hello-perftest
  git init
  ```

- Generate a new empty package which will contain our main
  application/library. Use the `ament_cmake` build system which
  simplifies integration with the ROS 2 package system. It is
  also possible to use "plain CMake" for more advanced use cases,
  and Python packages can use Python's `distutils` for a more "native"
  toolchain.

  ```sh
  ros2 pkg create --build-type ament_cmake hello_perftest
  ```

- The command will generate a empty package stub in the current  
  directory (NOTE: if you don't already know/have installed `tree`,
  I highly recommend it: `sudo apt install tree`):

  ```sh
  tree hello_perftest
  ```

  The command should produce the following output:
  
  ```txt
  hello_perftest
  ├── CMakeLists.txt
  ├── include
  │   └── hello_perftest
  ├── package.xml
  └── src
  ```

- Add a [`README.md`](README.md) file to the root of the repository, e.g.:

  ```sh
  printf -- "# An example ROS 2 C++ Project

  Getting started with ROS 2!
  " > README.md
  ```

- Edit/add the metadata fields in [`hello_perftest/package.xml`](hello_perftest/package.xml), for example
  by updating these fields (some of these fields may also be customized with
  arguments to `ros2 pkg create`, see `ros2 pkg create -h`):

  - `<version>`
  - `<description>`
  - `<maintainer>`.
  - `<author>` (optional, only needed if different from `<maintainer>`)
  - `<license>`

- Generate an "interfaces package", i.e. a ROS 2 package which will only
  contain definitions for interfaces used by our main ROS 2 package (messages,
  services, actions). It is a ROS 2 "best practice" to separate interfaces in
  separate packages to make it easier to share them across multiple projects.

  ```sh
  ros2 pkg create --build-type ament_cmake hello_perftest_interfaces
  ```

- Edit [`hello_perftest_interfaces/package.xml`](hello_perftest_interfaces/package.xml).

  Beside the fields that were mentioned earlier, let's add a dependency on
  interface package `std_msgs` so that we may reference those type definitions in our own types.
  
  ROS 2 packages which contains interfaces must also be made part of the
  `rosidl_interface_packages` group.
  
  Finally, building the interfaces will require the use of the available code
  generators, which are provided by package `rosidl_default_generators` (which
  is only required at build time), while running the generated code will
  require package `rosidl_default_runtime` to be available in the runtime
  environment.

  ```xml
  <package format="3">
    <name>hello_perftest_interfaces</name>
    <!-- ... -->
    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <depend>std_msgs</depend>

    <exec_depend>rosidl_default_runtime</exec_depend>

    <member_of_group>rosidl_interface_packages</member_of_group>
    <!-- ... -->
  </package>

  ```

- Create a directory where to store custom message definitions. Message
  definitions for a package are typically stored in a `msg/` subdirectory.

  ```sh
  mkdir hello_perftest_interfaces/msg
  ```

- Define an example message type using ROS IDL. Create
  [`hello_perftest_interfaces/msg/TestPayload.msg`](hello_perftest_interfaces/msg/TestPayload.msg) with the following contents:

  ```txt
  # A simple message used by our example ROS 2 package.

  # Metadata information about this payload
  std_msgs/Header header

  # An unbounded array of bytes
  byte[] data
  ```

- Modify [`hello_perftest_interfaces/CMakeLists.txt`](hello_perftest_interfaces/CMakeLists.txt) to process the custom
  message definitions, generate the required source code, built it into a
  libraries, and install it so that the types may be used by other packages.

  - Load packages `std_msgs` and `rosidl_default_generators`. Dependencies
    are loaded usign CMake's `find_package()` function. All required search
    paths are already set up by `colcon`/`ament` build system.

    ```cmake
    find_package(std_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    ```

  - Use function [`rosidl_generate_interfaces()`](https://github.com/ros2/rosidl/blob/master/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake) to
    generate type supports for our custom messages:

    ```cmake
    rosidl_generate_interfaces(${PROJECT_NAME}
      msg/TestPayload.msg
      DEPENDENCIES std_msgs
    )
    ```
  
  - Use some of the functions provided by `ament` to export runtime
    dependencies, and to declare the current source directory as a ROS 2
    package:

    ```cmake
    ament_export_dependencies(
      rosidl_default_runtime
      std_msgs
    )

    ament_package()
    ```

- Delete unused directories in `hello_perftest_interfaces`:

  ```sh
  rm -r hello_perftest_interfaces/{include,src}
  ```

- Edit [`hello_perftest/package.xml`](hello_perftest/package.xml) and add a
  dependency from `hello_perftest_interfaces`. Since we are going to use the
  ROS 2 C++ API, also add a dependency from `rclcpp`. We will also be taking
  advantage of the `rclcpp_components` package.

  ```xml
  <package format="3">
    <name>hello_perftest</name>
    <!-- ... -->
    <depend>hello_perftest_interfaces</depend>

    <depend>rclcpp</depend>

    <depend>rclcpp_components</depend>
    <!-- ... -->
  </package>

  ```

- Define `hello_perftest::LatencyNode` as an abstract subclass of 
  `rclcpp::Node` to represent the common structure of a component
  participating in a latency test.

  Extend this class into two concrete classes, 
  `hello_perftest::LatencyPublisher` and `hello_perftest::LatencySubscriber`.

  - See [hello_perftest/latency.hpp](hello_perftest/include/hello_perftest/latency_pub.hpp) for the definition of the classes which is very simple.

    The base class creates a publisher and a subcriber on two custom topics.
    The endpoints use custom Quality of Service. The subscriber is associated
    with a virtual callback to be implemented by each concrete subclass.

    The nodes declare some "parameters" to control some configuration options.
    This is just to showcase the usage of [ROS 2 parameters](https://docs.ros.org/en/galactic/Concepts/About-ROS-2-Parameters.html) which would allow
    the application to be dynamically reconfigured using YAML or command line
    arguments.

  - Each concrete class is registered as an "rclcpp component" in
    [latency.cpp](hello_perftest/src/latency/latency.cpp). This will allow us
    to build all the nodes into a single shared library, and then automatically
    generate executables to "spin them up" in an independent process using
    some CMake function.

  - The nodes can also be used in a custom, hand-written `main()`. In this case
    it is possible to build the nodei instances passing any number of custom 
    arguments to their constructor (whereas the node classes must have a 
    constructor which accepts a single argument of type `rclcpp::NodeOptions` 
    in order to be used as components).

  - The publisher uses the ROS graph events to wait for the discovery of the
    expected number of subscribers before beginning the test.

    - The nodes could also be turned into "managed nodes" by making them extend
      `rclcpp_lifecycle::LifecycleNode` instead of `rclcpp::Node`, and 
      by mapping various test actions to "lifecycle transitions" which may be
      then controlled by an external client
      (see this [design document](https://design.ros2.org/articles/node_lifecycle.html) and this [example](https://github.com/ros2/demos/tree/master/lifecycle) for more information on managed nodes).

- Modify `hello_perftest/CMakeLists.txt` to build the example latency  
  application.

  - Load the package with custom interfaces and other dependencies:

    ```cmake
    find_package(rclcpp)
    find_package(rclcpp_components)
    find_package(${PROJECT_NAME}_interfaces REQUIRED)
    ```
  
  - Generate a library with for the node components. We build a single
    shared library for the whole package.

    ```cmake
    add_library(${PROJECT_NAME} SHARED
      src/latency/latency.cpp
      include/hello_perftest/latency.hpp
    )
    ament_target_dependencies(${PROJECT_NAME}
      rclcpp
      rclcpp_components
      ${PROJECT_NAME}_interfaces
    )
    target_include_directories(${PROJECT_NAME}
      PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    )
    install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    install(DIRECTORY include/
      DESTINATION include
    )
    ```

  - Generate two executables from the registered node components:

    ```cmake
    rclcpp_components_register_node(${PROJECT_NAME}
      PLUGIN "hello_perftest::LatencySubscriber"
      EXECUTABLE latency_sub
    )

    rclcpp_components_register_node(${PROJECT_NAME}
      PLUGIN "hello_perftest::LatencyPublisher"
      EXECUTABLE latency_pub
    )
    ```
  
  - Alternatively, generate two executables using hand-written `main()` 
    functions:

    ```cmake
    add_executable(latency_sub_main
      src/latency/latency_subscriber_main.cpp
    )
    target_link_libraries(latency_sub_main ${PROJECT_NAME})
    install(TARGETS latency_sub_main
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    add_executable(latency_pub_main
      src/latency/latency_publisher_main.cpp
    )
    target_link_libraries(latency_pub_main ${PROJECT_NAME})
    install(TARGETS latency_pub_main
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    ```

- Commit files to `git`:

  ```sh
  git add -A
  git commit -m "Initial commit".
  ```

- Create a workspace and build the packages:

  ```sh
  # Create a workspace directory
  mkdir ws-hello-perftest
  cd ws-hello-perftest

  # Create a symlink to the repository's *root*,
  # not to the `hello_perftest` package.
  ln -s ../hello-perftest .

  # Build everything with colcon
  colcon build --symlink-install
  ```

- Run test applications:

  - Load the built workspace:

    ```sh
    source install/setup.bash
    ```

  - Components can be run using `ros2 run`:

    ```sh
    # Start a subscriber
    ros2 run hello_perftest latency_sub

    # Start a publisher
    ros2 run hello_perftest latency_pub
    ```

  - Hand-written executable can be run by hand:

    ```sh
    install/hello_perftest/bin/latency_pub_main

    install/hello_perftest/bin/latency_sub_main
    ```

  - Since the nodes use parameters, they can be configured from the
    command-line. The node name can also be changed when spawning multiple
    nodes (not necessary, but it allows the nodes to appear with different
    names in the ROS graph, e.g. if inspected with `rqt`).

    ```sh
    # Change number of expected subscribers
    ros2 run hello_perftest latency_pub --ros-args -p subscribers_count:=3

    # Start subscribers with different names
    ros2 run hello_perftest latency_sub --ros-args -r __node:=latency_sub_1

    ros2 run hello_perftest latency_sub --ros-args -r __node:=latency_sub_2

    ros2 run hello_perftest latency_sub --ros-args -r __node:=latency_sub_3

    ```

  - Parameters could also be changed using a YAML files. For example, saving
    the following as `params.yml`:

    ```yaml
    latency_pub:
      ros__parameters:
        subscribers_count: 2
    ```

    You can specify a parameters file with `--params-file`:

    ```sh
    ros2 run hello_perftest latency_pub --ros-args --params-file params.yml
    ```

  - The example also includes a process which spawns both nodes:

    ```sh
    ./install/hello_perftest/bin/latency_single
    ```
