from setuptools import find_packages, setup


package_name = "inspection_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/straight_single_span.yaml"]),
    ],
    scripts=["scripts/run_baseline_scenario.py", "scripts/run_scenario_suite.py", "scripts/run_ablation.py", "scripts/run_ablation_batch.py", "scripts/validate_artifacts.py"],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Geometry and coverage primitives for the transmission-line inspection prototype.",
    license="Proprietary",
    entry_points={},
)
