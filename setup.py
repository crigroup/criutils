#!/usr/bin/env python

from distutils.core import setup

package_name='criutils'
setup(
        name=package_name,
        version='0.1.4',
        description='The criutils package',
        author='Francisco Suarez-Ruiz',
        author_email='fsuarez6@gmail.com',
        url='http://wiki.ros.org/criutils',
        packages=['src/' + package_name],
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ],
        )
