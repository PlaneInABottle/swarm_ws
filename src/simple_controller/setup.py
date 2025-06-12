import os
from glob import glob
from setuptools import find_packages, setup

package_name = "simple_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # === LAUNCH DOSYALARINI EKLEMEK İÇİN BU SATIRI EKLEYİN ===
        # Bu satır, 'launch' klasöründeki tüm .py uzantılı dosyaları
        # 'install/simple_controller/share/simple_controller/launch' dizinine kopyalar.
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name), glob("fastdds.xml")),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mirza",
    maintainer_email="y.mirza.altay@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Mevcut controller düğümünüz
            "core_drone_controller_node = simple_controller.nodes.core_drone_controller_node:main",
            # === ORCHESTRATOR DÜĞÜMÜNÜ EKLEMEK İÇİN BU SATIRI EKLEYİN ===
            "swarm_orchestrator_node = simple_controller.nodes.swarm_orchestrator_node:main",
            'swarm_state_publisher_node = simple_controller.nodes.swarm_state_publisher_node:main',
            # Formasyon Yönetici Node'u
            'formation_manager_node = simple_controller.nodes.formation_manager_node:main',
        ],
    },
)