from setuptools import setup

package_name = 'image_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hossein',
    maintainer_email='hsa150@sfu.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = image_segmentation.image_segmentation_subpub:main',
            'service = image_segmentation.image_segmentation_service:main',
            'service_v2 = image_segmentation.image_segmentation_service_v2:main',
            'client = image_segmentation.client:main',
        ],
    },
)
