"""cppmake auto-generates python code
"""
import setuptools

description = "Python Super CMake"

setuptools.setup(
    name='pymake',
    version='1.0',
    license='MIT',
    long_description=__doc__,
    url='panikul.am',
    author_email='jpanikul@gmail.com',
    packages=setuptools.find_packages(),
    description=description,
    keywords="cmake make stupid",
    platforms='any',
    zip_safe=True
)
