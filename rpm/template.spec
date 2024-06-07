%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/jazzy/.*$
%global __requires_exclude_from ^/opt/ros/jazzy/.*$

Name:           ros-jazzy-mavros-extras
Version:        2.8.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS mavros_extras package

License:        GPLv3 and LGPLv3 and BSD
URL:            http://wiki.ros.org/mavros_extras
Source0:        %{name}-%{version}.tar.gz

Requires:       GeographicLib
Requires:       GeographicLib-devel
Requires:       eigen3-devel
Requires:       ros-jazzy-diagnostic-msgs
Requires:       ros-jazzy-diagnostic-updater
Requires:       ros-jazzy-eigen-stl-containers
Requires:       ros-jazzy-eigen3-cmake-module
Requires:       ros-jazzy-geographic-msgs
Requires:       ros-jazzy-geometry-msgs
Requires:       ros-jazzy-libmavconn
Requires:       ros-jazzy-mavlink
Requires:       ros-jazzy-mavros
Requires:       ros-jazzy-mavros-msgs
Requires:       ros-jazzy-message-filters
Requires:       ros-jazzy-nav-msgs
Requires:       ros-jazzy-pluginlib
Requires:       ros-jazzy-rclcpp
Requires:       ros-jazzy-rclcpp-components
Requires:       ros-jazzy-rcpputils
Requires:       ros-jazzy-rosidl-default-runtime
Requires:       ros-jazzy-sensor-msgs
Requires:       ros-jazzy-std-msgs
Requires:       ros-jazzy-std-srvs
Requires:       ros-jazzy-tf2-eigen
Requires:       ros-jazzy-tf2-ros
Requires:       ros-jazzy-trajectory-msgs
Requires:       ros-jazzy-urdf
Requires:       ros-jazzy-visualization-msgs
Requires:       ros-jazzy-yaml-cpp-vendor
Requires:       yaml-cpp-devel
Requires:       ros-jazzy-ros-workspace
BuildRequires:  GeographicLib
BuildRequires:  GeographicLib-devel
BuildRequires:  eigen3-devel
BuildRequires:  ros-jazzy-ament-cmake
BuildRequires:  ros-jazzy-ament-cmake-python
BuildRequires:  ros-jazzy-angles
BuildRequires:  ros-jazzy-diagnostic-msgs
BuildRequires:  ros-jazzy-diagnostic-updater
BuildRequires:  ros-jazzy-eigen-stl-containers
BuildRequires:  ros-jazzy-eigen3-cmake-module
BuildRequires:  ros-jazzy-geographic-msgs
BuildRequires:  ros-jazzy-geometry-msgs
BuildRequires:  ros-jazzy-libmavconn
BuildRequires:  ros-jazzy-mavlink
BuildRequires:  ros-jazzy-mavros
BuildRequires:  ros-jazzy-mavros-msgs
BuildRequires:  ros-jazzy-message-filters
BuildRequires:  ros-jazzy-nav-msgs
BuildRequires:  ros-jazzy-pluginlib
BuildRequires:  ros-jazzy-rclcpp
BuildRequires:  ros-jazzy-rclcpp-components
BuildRequires:  ros-jazzy-rcpputils
BuildRequires:  ros-jazzy-sensor-msgs
BuildRequires:  ros-jazzy-std-msgs
BuildRequires:  ros-jazzy-std-srvs
BuildRequires:  ros-jazzy-tf2-eigen
BuildRequires:  ros-jazzy-tf2-ros
BuildRequires:  ros-jazzy-trajectory-msgs
BuildRequires:  ros-jazzy-urdf
BuildRequires:  ros-jazzy-visualization-msgs
BuildRequires:  ros-jazzy-yaml-cpp-vendor
BuildRequires:  yaml-cpp-devel
BuildRequires:  ros-jazzy-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  gmock-devel
BuildRequires:  gtest-devel
BuildRequires:  ros-jazzy-ament-cmake-gmock
BuildRequires:  ros-jazzy-ament-cmake-gtest
BuildRequires:  ros-jazzy-ament-lint-auto
BuildRequires:  ros-jazzy-ament-lint-common
%endif

%description
Extra nodes and plugins for MAVROS.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/jazzy" \
    -DAMENT_PREFIX_PATH="/opt/ros/jazzy" \
    -DCMAKE_PREFIX_PATH="/opt/ros/jazzy" \
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
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/jazzy

%changelog
* Fri Jun 07 2024 Vladimir Ermakov <vooon341@gmail.com> - 2.8.0-1
- Autogenerated by Bloom

* Thu Apr 18 2024 Vladimir Ermakov <vooon341@gmail.com> - 2.6.0-3
- Autogenerated by Bloom

* Wed Mar 06 2024 Vladimir Ermakov <vooon341@gmail.com> - 2.6.0-2
- Autogenerated by Bloom

