Name:           ros-kinetic-libmavconn
Version:        0.23.1
Release:        0%{?dist}
Summary:        ROS libmavconn package

Group:          Development/Libraries
License:        GPLv3
URL:            http://wiki.ros.org/mavros
Source0:        %{name}-%{version}.tar.gz

Requires:       boost-devel
Requires:       console-bridge-devel
Requires:       ros-kinetic-mavlink
BuildRequires:  boost-devel
BuildRequires:  console-bridge-devel
BuildRequires:  gtest-devel
BuildRequires:  ros-kinetic-catkin
BuildRequires:  ros-kinetic-mavlink

%description
MAVLink communication library. This library provide unified connection handling
classes and URL to connection object mapper. This library can be used in
standalone programs.

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_LIBDIR="lib" \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/kinetic" \
        -DCMAKE_PREFIX_PATH="/opt/ros/kinetic" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/kinetic

%changelog
* Tue Feb 27 2018 Vladimir Ermakov <vooon341@gmail.com> - 0.23.1-0
- Autogenerated by Bloom

* Sat Feb 03 2018 Vladimir Ermakov <vooon341@gmail.com> - 0.23.0-0
- Autogenerated by Bloom

* Mon Dec 11 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.22.0-0
- Autogenerated by Bloom

* Thu Nov 16 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.21.5-0
- Autogenerated by Bloom

* Wed Nov 01 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.21.4-0
- Autogenerated by Bloom

* Sat Oct 28 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.21.3-0
- Autogenerated by Bloom

* Mon Sep 25 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.21.2-0
- Autogenerated by Bloom

* Fri Sep 22 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.21.1-0
- Autogenerated by Bloom

* Thu Sep 14 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.21.0-0
- Autogenerated by Bloom

* Mon Aug 28 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.20.1-0
- Autogenerated by Bloom

* Sat Aug 26 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.20.0-1
- Autogenerated by Bloom

* Wed Aug 23 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.20.0-0
- Autogenerated by Bloom

* Fri May 05 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.19.0-0
- Autogenerated by Bloom

* Fri Feb 24 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.18.7-0
- Autogenerated by Bloom

* Tue Feb 07 2017 Vladimir Ermakov <vooon341@gmail.com> - 0.18.6-0
- Autogenerated by Bloom

* Mon Dec 12 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.5-0
- Autogenerated by Bloom

* Fri Nov 11 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.4-0
- Autogenerated by Bloom

* Thu Jul 07 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.3-1
- Autogenerated by Bloom

* Thu Jul 07 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.3-0
- Autogenerated by Bloom

* Thu Jun 30 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.2-0
- Autogenerated by Bloom

* Fri Jun 24 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.1-0
- Autogenerated by Bloom

* Thu Jun 23 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.18.0-0
- Autogenerated by Bloom

* Fri May 20 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.17.3-0
- Autogenerated by Bloom

* Sun May 15 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.17.2-1
- Autogenerated by Bloom

* Fri Apr 29 2016 Vladimir Ermakov <vooon341@gmail.com> - 0.17.2-0
- Autogenerated by Bloom

