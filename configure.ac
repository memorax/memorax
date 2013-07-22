AC_INIT([Memorax], [0.1.1], [carl.leonardsson@it.uu.se])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
: {$CXXFLAGS=""}
AC_PROG_CXX
if test xxx$GXX = xxxyes; then
   CXXFLAGS="$CXXFLAGS -O2"
fi
USEDNDEBUG='yes'
AC_ARG_ENABLE([debug-assert],
        [  --enable-debug-assert  Enable cassert; do not compile with -DNDEBUG.],
        [USEDNDEBUG='no'])
if test xxx$USEDNDEBUG = xxxyes; then
   CXXFLAGS="$CXXFLAGS -DNDEBUG"
fi
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([
  Makefile
  src/Makefile
  doc/Makefile
])
AC_LANG([C++])
AC_MSG_CHECKING([for lambdas])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[#include <functional>]],
                                   [[int j = 42; std::function<int(int)> f = [j](int i){ return i+j; }]])],
  [AC_MSG_RESULT([yes])],
  [AC_MSG_RESULT([no]) ; 
   AC_MSG_CHECKING([for lambdas with switch -std=c++0x]) ;
   OLDCXXFLAGS=$CXXFLAGS ;
   CXXFLAGS="$CXXFLAGS -std=c++0x" ;
   AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[#include <functional>]],
                                       [[int j = 42; std::function<int(int)> f = [j](int i){ return i+j; }]])],
     [AC_MSG_RESULT([yes])],
     [AC_MSG_FAILURE([Failed to compile lambda expression. Does your compiler support C++0x?])])])
AC_MSG_CHECKING([for automatic type inference])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[#include <list>]],
                                   [[std::list<int> l; for(auto it = l.begin(); it != l.end(); ++it) ;]])],
                  [AC_MSG_RESULT([yes])],
                  [AC_MSG_RESULT([no]) ; AC_MSG_FAILURE([Failed to compile auto-typed declaration. Does your compiler support C++0x?])])
AC_MSG_CHECKING([for gmpxx])
OLDLIBS=$LIBS
LIBS="$LIBS -lgmpxx -lgmp"
AC_LINK_IFELSE([AC_LANG_PROGRAM(
                    [[#include <gmpxx.h>
                      #include <iostream>]],
                    [[mpz_class a, b, c; a = 1234; b = "-5678"; c = a+b;]])],
                  [AC_MSG_RESULT([yes]);
                   HAVE_LIBGMPXX='yes';
                   AC_DEFINE([HAVE_LIBGMPXX],[1],[Define to 1 if you have the `gmpxx' library (-lgmpxx).]) ],
                  [AC_MSG_RESULT([no]) ;
                   HAVE_LIBGMPXX='no';
                   AC_DEFINE([HAVE_LIBGMPXX],[0],[Define to 1 if you have the `gmpxx' library (-lgmpxx).]) ])
HAVE_MATHSAT='no'
if test xxx$HAVE_LIBGMPXX = xxxyes ; then
   AC_CHECK_HEADER([gmp.h],
                   [],
                   [AC_MSG_FAILURE([Could not find header file gmp.h.])])
   LIBS="$OLDLIBS -lmathsat -lgmpxx -lgmp"
   AC_CHECK_HEADER([mathsat.h],
                   [HAVE_MATHSAT_H='yes'],
                   [HAVE_MATHSAT_H='no';
                    AC_MSG_RESULT([Could not find header file mathsat.h.])])
   if test xxx$HAVE_MATHSAT_H = xxxyes ; then
      AC_CHECK_LIB([mathsat],[msat_create_env],
                   [HAVE_MATHSAT='yes'],
                   [])
      if test xxx$HAVE_MATHSAT = xxxyes ; then
         AC_MSG_CHECKING([whether mathsat is version 4 or 5])
         AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[#include <stdio.h>
                                              #include <gmp.h>
                                              #include <mathsat.h>]],
                                            [[msat_create_config();]])],
                           [AC_MSG_RESULT([assuming version 5]) ; 
                            AC_DEFINE([MATHSAT_VERSION],[5],[Using mathsat version 4 or 5?])
                            ],
                           [AC_MSG_RESULT([assuming version 4]) ; 
                            AC_DEFINE([MATHSAT_VERSION],[4],[Using mathsat version 4 or 5?])
                            ])
      fi
   fi
fi

if test xxx$HAVE_MATHSAT != xxxyes ; then
   HAVE_MATHSAT='no';
   LIBS="$OLDLIBS"
   AC_DEFINE([HAVE_LIBMATHSAT],[0],[Define to 1 if you have the `mathsat' library (-lmathsat).])
   AC_MSG_WARN([Unable to compile with MathSAT.]);
   AC_MSG_WARN([This will disable predicate abstraction.]);
else
  AC_DEFINE([HAVE_LIBMATHSAT],[1],[Define to 1 if you have the `mathsat' library (-lmathsat).])
fi

m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

AC_SUBST([MANUALDEP])
AC_SUBST([MANUALBUILDRULE])
AC_SUBST([FDLDEP])
AC_SUBST([FDLBUILDRULE])
AC_SUBST([MANUALCLEANRULE])
MANUALDEP='manual.tex fdl.pdf';
FDLDEP='fdl-1.3.tex'
MANUALBUILDRULE='pdflatex -interaction=batchmode manual.tex && bibtex manual.aux && pdflatex -interaction=batchmode manual.tex && pdflatex -interaction=batchmode manual.tex';
FDLBUILDRULE='pdflatex -interaction=batchmode fdl-1.3.tex && pdflatex -interaction=batchmode fdl-1.3.tex && mv fdl-1.3.pdf fdl.pdf'
MANUALCLEANRULE='rm -f manual.aux manual.bbl manual.blg manual.log manual.out manual.toc manual.pdf fdl-1.3.out fdl-1.3.aux fdl-1.3.log fdl.pdf'
AC_CHECK_FILE([doc/manual.tex],
              [AC_CHECK_FILE([doc/biblio.bib],
                             [],
                             [AC_MSG_FAILURE([Found manual.tex but not biblio.bib.])])],
              [MANUALDEP='';
               MANUALBUILDRULE='echo "Not building manual.pdf."';
               MANUALCLEANRULE='echo "Not removing manual.pdf."'])

AC_SUBST([GUI])
AM_PATH_PYTHON([2.6],,[:])
if test xxx$PYTHON != xxx: ; then
   AC_MSG_CHECKING(for python module Tkinter)
   if echo 'import Tkinter' | $PYTHON 2>/dev/null ; then
      AC_MSG_RESULT([yes]) ;
      HAVE_TKINTER='yes' ;
   else
      AC_MSG_RESULT([no]) ;
      AC_MSG_WARN([Tkinter is necessary for the graphical interface.]) ;
      AC_MSG_WARN([Hint: On Debian based systems, python-tk is the package to install.])
      HAVE_TKINTER='no' ;
   fi
fi
AC_CHECK_PROG([HAVE_SED],[sed],[yes],[no])
if test xxx$PYTHON != xxx: && test xxx$HAVE_SED = xxxyes && test xxx$HAVE_TKINTER = xxxyes ; then
   AC_MSG_RESULT([Will additionally install graphical interface.])
   GUI='memorax-gui'
else
   AC_MSG_WARN([Not installing graphical interface.])
   GUI=''
fi

AC_CHECK_PROG([HAVE_DOT],[dot],[yes],[no])
if test xxx$HAVE_DOT != xxxyes ; then
   AC_MSG_WARN([Unable to find dot; will not be able to graphically represent automata.])
   AC_MSG_WARN([Install Graphviz to enable graphical representation of automata.])
fi

AC_OUTPUT


## AC_ARG_ENABLE - för att lägga till nya argument till ./configure


