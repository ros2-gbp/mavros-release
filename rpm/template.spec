%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-mavros
Version:        2.6.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS mavros package

License:        GPLv3 and LGPLv3 and BSD
URL:            http://wiki.ros.org/mavros
Source0:        %{name}-%{version}.tar.gz

Requires:       GeographicLib
Requires:       GeographicLib-devel
Requires:       console-bridge-devel
Requires:       eigen3-devel
Requires:       python%{python3_pkgversion}-click
Requires:       ros-humble-diagnostic-msgs
Requires:       ros-humble-diagnostic-updater
Requires:       ros-humble-eigen-stl-containers
Requires:       ros-humble-eigen3-cmake-module
Requires:       ros-humble-geographic-msgs
Requires:       ros-humble-geometry-msgs
Requires:       ros-humble-libmavconn
Requires:       ros-humble-mavlink
Requires:       ros-humble-mavros-msgs
Requires:       ros-humble-message-filters
Requires:       ros-humble-nav-msgs
Requires:       ros-humble-pluginlib
Requires:       ros-humble-rclcpp
Requires:       ros-humble-rclcpp-components
Requires:       ros-humble-rclpy
Requires:       ros-humble-rcpputils
Requires:       ros-humble-rosidl-default-runtime
Requires:       ros-humble-sensor-msgs
Requires:       ros-humble-std-msgs
Requires:       ros-humble-std-srvs
Requires:       ros-humble-tf2-eigen
Requires:       ros-humble-tf2-ros
Requires:       ros-humble-trajectory-msgs
Requires:       ros-humble-ros-workspace
BuildRequires:  GeographicLib
BuildRequires:  GeographicLib-devel
BuildRequires:  console-bridge-devel
BuildRequires:  eigen3-devel
BuildRequires:  ros-humble-ament-cmake
BuildRequires:  ros-humble-ament-cmake-python
BuildRequires:  ros-humble-angles
BuildRequires:  ros-humble-diagnostic-msgs
BuildRequires:  ros-humble-diagnostic-updater
BuildRequires:  ros-humble-eigen-stl-containers
BuildRequires:  ros-humble-eigen3-cmake-module
BuildRequires:  ros-humble-geographic-msgs
BuildRequires:  ros-humble-geometry-msgs
BuildRequires:  ros-humble-libmavconn
BuildRequires:  ros-humble-mavlink
BuildRequires:  ros-humble-mavros-msgs
BuildRequires:  ros-humble-message-filters
BuildRequires:  ros-humble-nav-msgs
BuildRequires:  ros-humble-pluginlib
BuildRequires:  ros-humble-rclcpp
BuildRequires:  ros-humble-rclcpp-components
BuildRequires:  ros-humble-rcpputils
BuildRequires:  ros-humble-sensor-msgs
BuildRequires:  ros-humble-std-msgs
BuildRequires:  ros-humble-std-srvs
BuildRequires:  ros-humble-tf2-eigen
BuildRequires:  ros-humble-tf2-ros
BuildRequires:  ros-humble-trajectory-msgs
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  gmock-devel
BuildRequires:  gtest-devel
BuildRequires:  ros-humble-ament-cmake-gmock
BuildRequires:  ros-humble-ament-cmake-gtest
BuildRequires:  ros-humble-ament-cmake-pytest
BuildRequires:  ros-humble-ament-lint-auto
BuildRequires:  ros-humble-ament-lint-common
%endif

%description
MAVROS -- MAVLink extendable communication node for ROS with proxy for Ground
Control Station.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Sat Sep 09 2023 Vladimir Ermakov <vooon341@gmail.com> - 2.6.0-1
- Autogenerated by Bloom

* Fri May 05 2023 Vladimir Ermakov <vooon341@gmail.com> - 2.5.0-1
- Autogenerated by Bloom

* Sat Dec 31 2022 Vladimir Ermakov <vooon341@gmail.com> - 2.4.0-1
- Autogenerated by Bloom

* Sat Sep 24 2022 Vladimir Ermakov <vooon341@gmail.com> - 2.3.0-1
- Autogenerated by Bloom

* Mon Jun 27 2022 Vladimir Ermakov <vooon341@gmail.com> - 2.2.0-1
- Autogenerated by Bloom

* Tue Apr 19 2022 Vladimir Ermakov <vooon341@gmail.com> - 2.1.0-3
- Autogenerated by Bloom

* Tue Feb 08 2022 Vladimir Ermakov <vooon341@gmail.com> - 2.1.0-2
- Autogenerated by Bloom

