%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/kilted/.*$
%global __requires_exclude_from ^/opt/ros/kilted/.*$

%global __cmake_in_source_build 1

Name:           ros-kilted-mavros
Version:        2.9.0
Release:        2%{?dist}%{?release_suffix}
Summary:        ROS mavros package

License:        GPLv3 and LGPLv3 and BSD
URL:            http://wiki.ros.org/mavros
Source0:        %{name}-%{version}.tar.gz

Requires:       GeographicLib
Requires:       GeographicLib-devel
Requires:       console-bridge-devel
Requires:       eigen3-devel
Requires:       python%{python3_pkgversion}-click
Requires:       ros-kilted-diagnostic-msgs
Requires:       ros-kilted-diagnostic-updater
Requires:       ros-kilted-eigen-stl-containers
Requires:       ros-kilted-eigen3-cmake-module
Requires:       ros-kilted-geographic-msgs
Requires:       ros-kilted-geometry-msgs
Requires:       ros-kilted-libmavconn
Requires:       ros-kilted-mavlink
Requires:       ros-kilted-mavros-msgs
Requires:       ros-kilted-message-filters
Requires:       ros-kilted-nav-msgs
Requires:       ros-kilted-pluginlib
Requires:       ros-kilted-rclcpp
Requires:       ros-kilted-rclcpp-components
Requires:       ros-kilted-rclpy
Requires:       ros-kilted-rcpputils
Requires:       ros-kilted-rosidl-default-runtime
Requires:       ros-kilted-sensor-msgs
Requires:       ros-kilted-std-msgs
Requires:       ros-kilted-std-srvs
Requires:       ros-kilted-tf2-eigen
Requires:       ros-kilted-tf2-ros
Requires:       ros-kilted-trajectory-msgs
Requires:       ros-kilted-ros-workspace
BuildRequires:  GeographicLib
BuildRequires:  GeographicLib-devel
BuildRequires:  console-bridge-devel
BuildRequires:  eigen3-devel
BuildRequires:  ros-kilted-ament-cmake
BuildRequires:  ros-kilted-ament-cmake-python
BuildRequires:  ros-kilted-angles
BuildRequires:  ros-kilted-diagnostic-msgs
BuildRequires:  ros-kilted-diagnostic-updater
BuildRequires:  ros-kilted-eigen-stl-containers
BuildRequires:  ros-kilted-eigen3-cmake-module
BuildRequires:  ros-kilted-geographic-msgs
BuildRequires:  ros-kilted-geometry-msgs
BuildRequires:  ros-kilted-libmavconn
BuildRequires:  ros-kilted-mavlink
BuildRequires:  ros-kilted-mavros-msgs
BuildRequires:  ros-kilted-message-filters
BuildRequires:  ros-kilted-nav-msgs
BuildRequires:  ros-kilted-pluginlib
BuildRequires:  ros-kilted-rclcpp
BuildRequires:  ros-kilted-rclcpp-components
BuildRequires:  ros-kilted-rcpputils
BuildRequires:  ros-kilted-sensor-msgs
BuildRequires:  ros-kilted-std-msgs
BuildRequires:  ros-kilted-std-srvs
BuildRequires:  ros-kilted-tf2-eigen
BuildRequires:  ros-kilted-tf2-ros
BuildRequires:  ros-kilted-trajectory-msgs
BuildRequires:  ros-kilted-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  gmock-devel
BuildRequires:  gtest-devel
BuildRequires:  ros-kilted-ament-cmake-gmock
BuildRequires:  ros-kilted-ament-cmake-gtest
BuildRequires:  ros-kilted-ament-cmake-pytest
BuildRequires:  ros-kilted-ament-lint-auto
BuildRequires:  ros-kilted-ament-lint-common
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
if [ -f "/opt/ros/kilted/setup.sh" ]; then . "/opt/ros/kilted/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/kilted" \
    -DAMENT_PREFIX_PATH="/opt/ros/kilted" \
    -DCMAKE_PREFIX_PATH="/opt/ros/kilted" \
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
if [ -f "/opt/ros/kilted/setup.sh" ]; then . "/opt/ros/kilted/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kilted/setup.sh" ]; then . "/opt/ros/kilted/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/kilted

%changelog
* Tue Apr 22 2025 Vladimir Ermakov <vooon341@gmail.com> - 2.9.0-2
- Autogenerated by Bloom

* Thu Oct 10 2024 Vladimir Ermakov <vooon341@gmail.com> - 2.9.0-1
- Autogenerated by Bloom

* Fri Jun 07 2024 Vladimir Ermakov <vooon341@gmail.com> - 2.8.0-1
- Autogenerated by Bloom

* Wed Mar 06 2024 Vladimir Ermakov <vooon341@gmail.com> - 2.6.0-2
- Autogenerated by Bloom

