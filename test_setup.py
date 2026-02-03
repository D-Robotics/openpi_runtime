#!/usr/bin/env python3
"""
验证 setup.py 配置是否正确包含新的 ServerUtils 模块
"""

import sys
import os
from setuptools import find_packages, setup


def test_packages():
    """测试包发现"""
    print("=" * 50)
    print("测试包发现功能")
    print("=" * 50)

    packages = find_packages(exclude=["test"])
    print(f"发现的包: {len(packages)} 个")

    expected_packages = [
        "common",
        "common.ServerUtils",
        "common.msg",
        "common.utils",
        "openpi_runtime",
        "openpi_runtime.utils",
    ]

    print("\n期望的包:")
    for pkg in expected_packages:
        status = "✓" if pkg in packages else "✗"
        print(f"  {status} {pkg}")

    missing = set(expected_packages) - set(packages)
    if missing:
        print(f"\n❌ 缺失的包: {missing}")
        return False
    else:
        print("\n✅ 所有期望的包都被发现")
        return True


def test_imports():
    """测试关键模块导入"""
    print("\n" + "=" * 50)
    print("测试模块导入")
    print("=" * 50)

    try:
        # 添加项目路径
        project_root = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, project_root)

        # 测试 ServerUtils 导入
        from common.ServerUtils import Server

        print("✓ common.ServerUtils.Server 导入成功")

        # 测试实例化
        server = Server(8888)
        print("✓ Server 实例化成功")

        return True

    except Exception as e:
        print(f"✗ 导入测试失败: {e}")
        return False


def test_setup_py():
    """测试 setup.py 配置"""
    print("\n" + "=" * 50)
    print("测试 setup.py 配置")
    print("=" * 50)

    try:
        # 读取 setup.py 内容
        with open("setup.py", "r") as f:
            content = f.read()

        # 检查关键配置
        checks = [
            ("find_packages", "find_packages" in content),
            ("protobuf依赖", "'protobuf'" in content or '"protobuf"' in content),
            ("package_name", "openpi_runtime" in content),
        ]

        for check_name, result in checks:
            status = "✓" if result else "✗"
            print(f"{status} {check_name}")

        return all(result for _, result in checks)

    except Exception as e:
        print(f"✗ setup.py 测试失败: {e}")
        return False


def main():
    """主测试函数"""
    print("setup.py 配置验证测试")
    print("项目路径:", os.path.dirname(os.path.abspath(__file__)))

    tests = [
        ("包发现测试", test_packages),
        ("模块导入测试", test_imports),
        ("setup.py配置测试", test_setup_py),
    ]

    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append(result)
        except Exception as e:
            print(f"✗ {test_name} 异常: {e}")
            results.append(False)

    print("\n" + "=" * 50)
    print("测试结果总结")
    print("=" * 50)

    passed = sum(results)
    total = len(results)

    for i, (test_name, _) in enumerate(tests):
        status = "✅ PASS" if results[i] else "❌ FAIL"
        print(f"{status} {test_name}")

    print(f"\n总体结果: {passed}/{total} 测试通过")

    if passed == total:
        print("🎉 setup.py 配置验证成功！")
        print("新的 ServerUtils 模块已正确包含在包配置中。")
    else:
        print("❌ 部分测试失败，请检查配置。")

    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
