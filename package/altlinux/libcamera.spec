%define _libexecdir %_prefix/libexec

%def_disable test
%def_enable cam
%def_enable lc_compliance
%def_enable qcam
%def_disable check

Name: libcamera-neko
Version: 999
Release: alt1
Provides: libcamera = 2:%EVR
Obsoletes: libcamera < 2:%EVR

Summary: A complex camera support library for Linux
License: LGPL-2.1-or-later
Group: Video
Url: https://libcamera.org/

Source: %name-%version.tar

BuildRequires(pre): rpm-macros-meson
BuildRequires: gcc-c++ meson >= 0.56
BuildRequires: openssl boost-devel
BuildRequires: pkgconfig(gnutls)
BuildRequires: pkgconfig(gstreamer-1.0)
BuildRequires: pkgconfig(gstreamer-video-1.0)
BuildRequires: pkgconfig(gstreamer-allocators-1.0)
BuildRequires: pkgconfig(libevent_pthreads)
BuildRequires: pkgconfig(udev)
BuildRequires: pkgconfig(yaml-0.1)
BuildRequires: pkgconfig(libexif)
BuildRequires: pkgconfig(libtiff-4)
BuildRequires: pkgconfig(libdrm)
BuildRequires: pkgconfig(libjpeg)
BuildRequires: pkgconfig(sdl2)
BuildRequires: pkgconfig(libyuv) >= 0.0.1922
BuildRequires: python3(jinja2)
BuildRequires: python3(yaml)
BuildRequires: python3(ply)
%ifarch aarch64
BuildRequires: pkgconfig(libpisp) >= 1.4.0
%endif
%if_enabled qcam
BuildRequires: qt6-tools-devel
BuildRequires: pkgconfig(Qt6Core)
BuildRequires: pkgconfig(Qt6Gui)
BuildRequires: pkgconfig(Qt6OpenGL)
BuildRequires: pkgconfig(Qt6OpenGLWidgets)
BuildRequires: pkgconfig(Qt6Widgets)
%endif
BuildRequires: pkgconfig(gtest)

%description
An open source camera stack and framework for Linux, Android, and ChromeOS.

%package -n gst-plugins-%{name}1.0
Summary: A complex camera support library for Linux
Group: System/Libraries
Requires: %name = %EVR
Provides: gst-plugins-libcamera1.0 = 2:%EVR
Obsoletes: gst-plugins-libcamera1.0 < 2:%EVR

%description -n gst-plugins-%{name}1.0
An open source camera stack and framework for Linux, Android, and ChromeOS.
This package contains libcamera gstreamer plugin.

%package -n qcam-neko
Summary: A complex camera support library for Linux
Group: Video
Requires: %name = %EVR
Provides: qcam = 2:%EVR
Obsoletes: qcam < 2:%EVR

%description -n qcam-neko
An open source camera stack and framework for Linux, Android, and ChromeOS.
This package contains Qt-based libcamera utility.

%package devel
Summary: A complex camera support library for Linux
Group: Development/C
Requires: %name = %EVR
Provides: libcamera-devel = 2:%EVR
Obsoletes: libcamera-devel < 2:%EVR

%description devel
An open source camera stack and framework for Linux, Android, and ChromeOS.
This package contains development part of libcamera.

%package test-apps
Summary: Test applications from libcamera
Group: Video
Requires: %name = %EVR
Provides: libcamera-test-apps = 2:%EVR
Obsoletes: libcamera-test-apps < 2:%EVR

%description test-apps
This package provides "cam" and "lc_compliance" test apps from libcamera.

%prep
%setup

%define platdefs auto,virtual

%build
%add_optflags %(getconf LFS_CFLAGS)
%meson \
    -Dpipelines=%platdefs \
    -Dv4l2=true \
    -Dwerror=false \
    -Ddocumentation=disabled \
    %{subst_enable_meson_bool test test} \
    %{subst_enable_meson_feature cam cam} \
    %{subst_enable_meson_feature lc_compliance lc-compliance} \
    %{subst_enable_meson_feature qcam qcam}
%nil
%meson_build

%install
%meson_install
mkdir -p %buildroot%_libdir/libcamera %buildroot%_datadir/libcamera

%files
%{?_enable_test:%_libexecdir/libcamera/vimc_ipa_proxy}
%_bindir/libcamerify
%_bindir/libcamera-bug-report
%ifarch %ix86 x86_64
%_libexecdir/libcamera/ipu3_ipa_proxy
%endif
%ifarch aarch64
%_libexecdir/libcamera/raspberrypi_ipa_proxy
%_libexecdir/libcamera/rkisp1_ipa_proxy
%_libexecdir/libcamera/mali-c55_ipa_proxy
%endif
%ifarch armh
%_libexecdir/libcamera/raspberrypi_ipa_proxy
%_libexecdir/libcamera/rkisp1_ipa_proxy
%endif
%_libexecdir/libcamera/soft_ipa_proxy
%_libdir/libcamera
%_libdir/libcamera-base.so.*
%_libdir/libcamera.so.*
# moved to libexecdir since 0.1.0
%_libexecdir/libcamera/v4l2-compat.so
%_datadir/libcamera/

%files -n gst-plugins-%{name}1.0
%_libdir/gstreamer-1.0/*

%if_enabled qcam
%files -n qcam-neko
%_bindir/qcam
%endif

%files devel
%_includedir/libcamera
%_libdir/libcamera-base.so
%_libdir/libcamera.so
%_pkgconfigdir/libcamera-base.pc
%_pkgconfigdir/libcamera.pc

%files test-apps
%{?_enable_cam:%_bindir/cam}
%{?_enable_lc_compliance:%_bindir/lc-compliance}


%changelog
