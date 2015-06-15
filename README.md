Installation Instructions
=========================

Memorax can be compiled and installed on UNIX-like systems.

Requirements
------------

  1. A C++ compiler supporting C++11. For example g++ version 4.6 or
     higher.

  2. In order to run the graphical interface, python is required at a
     version of 2.6 or higher installed with tcl/tk of version 8.4 or
     higher. Python version 3 and above are currently not supported
     due to major changes in the Tkinter API.

  3. For predicate abstraction the MathSAT SMT solver as well as the
     library gmpxx are required. Memorax supports MathSAT 4 and
     MathSAT 5. MathSAT 4 is recommended. Memorax can be compiled
     without MathSAT and gmpxx, but will then not support predicate
     abstraction.

  4. To be able to graphically draw automata, Graphviz is required.

  If you acquired Memorax by other means than downloading the
  installable tarballs (e.g. cloning the git repository) then GNU
  autotools and a latex installation will also be required.

Basic Installation
------------------

   In the simplest case, Memorax can be installed with the following
   commands:

    $ tar xvf memorax-<version>.tar.gz
    $ cd memorax-<version>
    $ ./configure
    $ make
    $ make install

   In case you have no configure script, then you should refer to the
   next section.

Building the configure Script and Makefiles
-------------------------------------------

   The installable tarballs are provided with a configure script and
   Makefile templates. But in case you acquired the Memorax sources by
   e.g. cloning the git repository, it is necessary to build those
   files using GNU autotools. The easiest way to do this is as
   follows:

    $ cd memorax-<version>
    $ autoreconf --install

   This will automatically produce a configure script as well as
   Makefile templates, from the autotools template files configure.ac,
   Makefile.am, doc/Makefile.am, src/Makefile.am.

   After building the installation files, you should be able to
   proceed in the same way as above:

    $ ./configure
    $ make
    $ make install
    
Installation Options
--------------------

   The configure script is built with GNU autotools, and should accept
   the usual options and environment variables. This section outlines
   some of the typical use cases.

###Changing Installation Directory

   The command 'make install' will install Memorax, its graphical
   interface and its documentation in the directories which are
   standard on your system. To override this behavior add the switch
   --prefix to the './configure' command:

    $ ./configure --prefix=/your/desired/install/path

###Compiling with Predicate Abstraction Support

   To support predicate abstraction, Memorax must be compiled with
   MathSAT and gmpxx. Their header files and shared libraries must
   reside where they can be found by the compilation. If they are
   installed in non-standard locations, then the compilation can be
   directed to their location by appropriately specifying CPPFLAGS and
   LDFLAGS when invoking the './configure' command:

    $ ./configure CPPFLAGS='-I/path/to/mathsat/include' \
                  LDFLAGS='-L/path/to/mathsat/lib'

   If MathSAT and/or gmpxx are not found by the configure script, then
   Memorax will be installed without support for predicate abstraction.

###Specifying Compiler

   When the configure script is invoked, it will by GNU autotools
   magic determine which C++ compiler will be used during
   compilation. In case e.g. your default compiler does not support
   C++11, but you have the compiler g++-4.6 installed at a
   non-standard location you may want to override this. In order to do
   so, specify the path to g++-4.6 in CXX when invoking the
   './configure' command:

    $ ./configure CXX='/path/to/g++-4.6'

###Choosing the right Python version

   The autotools script will choose the highest version of Python it
   can find on your path. If that version happens to be Python 3 or
   above, the installation will warn you that Tkinter was not found,
   even when it is installed. In order to be able to use the GUI, you
   must manually specify the path to Python 2. For example:

    $ ./configure PYTHON=/usr/bin/python2

Troubleshooting
---------------

###MSatFailure

   In case you get the following error message when trying to use the
   PB abstraction:

    Error: MSatFailure: Program is not compiled with MathSAT.

   In order to use predicate abstraction, (e.g. the PB abstraction)
   the Memorax tool needs to be compiled with MathSAT. To solve the
   problem, install MathSAT on your system and then reinstall
   Memorax. In case the installation fails to find MathSAT (see the
   output from the configure script), then try the instructions in the
   section "Compiling with Predicate Abstraction Support" above.

License
=======

   The Memorax tool is licensed under GPLv3. See COPYING.

   Memorax is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Memorax is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

Contact / Bug Report
====================

   Feedback, questions or bug reports should be directed to Carl
   Leonardsson (carl.leonardsson@it.uu.se).
