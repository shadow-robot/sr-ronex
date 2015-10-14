#!/usr/bin/env python

# ####################################################################
# Copyright (c) 2013, Shadow Robot Company, All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library.
# ####################################################################


class DriverGenerator(object):
    """
    Generates a new RoNeX module driver using the templates.
    """

    _module_name = None
    _product_id = None
    _substitutions = {}

    def __init__(self, module_name, product_id):
        self._module_name = module_name
        self._product_id = product_id

        self.build_substitutions()

    def build_substitutions(self):
        """
        Creates a dictionary for the substitutions needed in the templates.
        """
        self._substitutions["AUTOMATIC_GENERATOR_FILE_NAME"] = "sr_board_" + self._module_name


if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()

    parser.add_argument("--module_name", "-m", type=str, required=True,
                        help="The name of the new RoNeX module (e.g. ADC, GIO, etc...)")
    parser.add_argument("--product_id", "-p", type=str, required=True,
                        help="The product id in hex (e.g. 0x2000002)")
    args = parser.parse_args()

    driver_generator = DriverGenerator(args.module_name, args.product_id)