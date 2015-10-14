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
        print "Generating a driver skeleton for the RoNeX module[" + product_id + "]: " + module_name

        self._module_name = module_name
        self._product_id = product_id

        self.build_substitutions()

        # generates the header
        header_path = "../include/sr_ronex_drivers/" + \
                      self._substitutions["AUTOMATIC_GENERATOR_FILE_NAME"]+".hpp"
        self.generate_cpp_code("templates/driver.hpp", header_path)

        # generates the cpp
        source_path = "../src/" + \
                      self._substitutions["AUTOMATIC_GENERATOR_FILE_NAME"]+".cpp"
        self.generate_cpp_code("templates/driver.cpp", source_path)

        # add the cpp to the CMakeLists
        self.generate_cmake()

        # add the required lines to the plugin.xml
        self.generate_plugin_xml()

    def build_substitutions(self):
        """
        Creates a dictionary for the substitutions needed in the templates.
        """
        # for cpp and hpp
        self._substitutions["AUTOMATIC_GENERATOR_FILE_NAME"] = "sr_board_" + self._module_name.lower()
        self._substitutions["AUTOMATIC_GENERATOR_REPLACE_MODULE_NAME"] = self._module_name.upper()
        # product id without the "0x"
        self._substitutions["AUTOMATIC_GENERATOR_REPLACE_PRODUCT_ID"] = self._product_id[2:]

        # for CMake
        self._substitutions["#AUTOMATIC_GENERATOR_INSERT_ABOVE"] = \
            "src/" + self._substitutions["AUTOMATIC_GENERATOR_FILE_NAME"] +\
            ".cpp\n    #AUTOMATIC_GENERATOR_INSERT_ABOVE"

        # for ethercat_device_plugin.xml
        # convert the product id to int
        self._substitutions["AUTOMATIC_GENERATOR_PRODUCT_ID"] = str(int(self._product_id, 16))
        # just a template
        self._substitutions["<!-- AUTOMATIC_GENERATOR_INSERT_ABOVE -->"] = \
            """<class name="sr_ronex_drivers/AUTOMATIC_GENERATOR_PRODUCT_ID" type="SrBoardAUTOMATIC_GENERATOR_REPLACE_MODULE_NAME" base_class_type="EthercatDevice">
    <description>
      RoNeX - AUTOMATIC_GENERATOR_REPLACE_MODULE_NAME module
    </description>
  </class>"""
        # replace the different infos from the template above
        self._substitutions["<!-- AUTOMATIC_GENERATOR_INSERT_ABOVE -->"] = \
            self._substitute(self._substitutions["<!-- AUTOMATIC_GENERATOR_INSERT_ABOVE -->"]) +\
            "\n  <!-- AUTOMATIC_GENERATOR_INSERT_ABOVE -->"

    def generate_cpp_code(self, template_path, new_file_path):
        """
        Opens the cpp templates, do the proper substitutions and write the result to the correct file.
        """
        print "Generating : " + new_file_path

        with open(template_path, "r") as template, \
                open(new_file_path, "w") as new_file:
            for line in template:
                new_file.write(self._substitute(line))

    def generate_cmake(self):
        """
        Adds the cpp file to the cmake.
        """
        print "Adding src/" + self._substitutions["AUTOMATIC_GENERATOR_FILE_NAME"] + ".cpp to CMakeLists.txt"

        new_lines = []
        with open("../CMakeLists.txt", "r") as cmake_read:
            for line in cmake_read:
                new_lines.append(self._substitute(line))

        with open("../CMakeLists.txt", "w") as cmake_write:
            cmake_write.writelines(new_lines)

    def generate_plugin_xml(self):
        """
        Adds the required lines to the plugin xml
        """
        print "Adding lines to the ethercat_device_plugins.xml"

        new_lines = []
        with open("../ethercat_device_plugin.xml", "r") as plugin_xml_read:
            for line in plugin_xml_read:
                new_lines.append(self._substitute(line))

        with open("../ethercat_device_plugin.xml", "w") as plugin_xml_write:
            plugin_xml_write.writelines(new_lines)

    def _substitute(self, line):
        """
        Substitutes the values from the _substitutions dict in the given line
        @param line a line from the template
        @return the modified line
        """
        for keyword, replace_with in self._substitutions.iteritems():
            line = line.replace(keyword, replace_with)
        return line


if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()

    parser.add_argument("--module_name", "-m", type=str, required=True,
                        help="The name of the new RoNeX module (e.g. ADC, GIO, etc...)")
    parser.add_argument("--product_id", "-p", type=str, required=True,
                        help="The product id in hex (e.g. 0x2000002)")
    args = parser.parse_args()

    driver_generator = DriverGenerator(args.module_name, args.product_id)