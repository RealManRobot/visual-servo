from setuptools import setup, find_packages

setup(
    name="sam",
    version="0.1.0",
    description="基于开源yolo的sam模型，使用图片的纹理特征信息，在使用点提示，框提示，或者无提示的情况下，得到分割图片的结果，对结果进行处理分析。",
    long_description=open('README.md', 'r', encoding='utf-8').read(),
    long_description_content_type="text/markdown",
    url="http://192.168.0.188:8090/ai_lab_rd02/ai_sdks/sam.git",
    # author="sam", # 作者
    # author_email="<EMAIL>", # 作者邮箱
    # license="MIT", # 许可证
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires='>=3.8',
    packages=find_packages(exclude=['tests']),
    install_requires=[
        "ultralytics~=8.2.54",
        "opencv-python~=4.10.0.84"
    ],
    include_package_data=True,
    zip_safe=False,
)
