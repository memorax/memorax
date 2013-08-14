/*
 * Copyright (C) 2013 Magnus Lång
 *
 * This file is part of Memorax.
 *
 * Memorax is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Memorax is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* Utilities for prettyprinting */
namespace PPUtils {
  template<class Outstream, class Sequence, class Separator>
  /* Prints sequence seq to stream os with separator sep between each element,
   * by using operator<<. */
  static void print_sequence_with_separator(Outstream &os, const Sequence &seq,
                                            const Separator &sep) {
    auto list_iter = seq.begin();
    while (list_iter != seq.end()) {
      os << *list_iter;
      if (++list_iter != seq.end()) os << sep;
    }
  };

  template<class Outstream, class Sequence, class Separator,
           class Printer, class... Params>
  /* Prints sequence seq to stream os with separator sep between each element,
   * by using a user specified printer function p. */
  static void print_sequence_with_separator(Outstream &os, const Sequence &seq,
                                            const Separator &sep, Printer &&p,
                                            Params... printer_params) {
    auto list_iter = seq.begin();
    while (list_iter != seq.end()) {
      p(os, *list_iter, printer_params...);
      if (++list_iter != seq.end()) os << sep;
    }
  };
}
