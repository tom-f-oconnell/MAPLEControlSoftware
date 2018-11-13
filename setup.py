from setuptools import setup

setup(
    name='maple',
    version='0.0.0',
    packages=['maple'],
    #package_data={'maple': ['MAPLE.cfg']},
    include_package_data=True,
    # TODO maybe keep script out of package then?
    scripts=['MAPLE-GUI.py']
)
