#!/usr/bin/env python3
"""
测试 Server 类的重构是否成功
"""

import sys
import os

# 添加项目路径
project_root = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, project_root)


def test_server_import():
    """测试 Server 类导入"""
    try:
        from common.ServerUtils import Server

        print("✓ Server 类导入成功")
        return True
    except Exception as e:
        print(f"✗ Server 类导入失败: {e}")
        return False


def test_server_instantiation():
    """测试 Server 类实例化"""
    try:
        from common.ServerUtils import Server

        server = Server(8888)
        print("✓ Server 实例化成功")
        print(f"  - 端口: {server.port}")
        print(f"  - 类型: {type(server)}")
        return True
    except Exception as e:
        print(f"✗ Server 实例化失败: {e}")
        return False


def test_server_methods():
    """测试 Server 类方法存在"""
    try:
        from common.ServerUtils import Server

        server = Server(8888)

        methods = [
            "listen",
            "start",
            "stop",
            "close",
            "send",
            "receive",
            "_build_header",
        ]
        for method in methods:
            if hasattr(server, method):
                print(f"✓ 方法 {method} 存在")
            else:
                print(f"✗ 方法 {method} 不存在")
                return False
        return True
    except Exception as e:
        print(f"✗ 测试方法失败: {e}")
        return False


def main():
    print("=" * 50)
    print("Server 类重构测试")
    print("=" * 50)

    tests = [
        ("导入测试", test_server_import),
        ("实例化测试", test_server_instantiation),
        ("方法测试", test_server_methods),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n[{test_name}]")
        result = test_func()
        results.append(result)

    print("\n" + "=" * 50)
    print("测试结果总结:")
    passed = sum(results)
    total = len(results)
    print(f"通过: {passed}/{total}")

    if passed == total:
        print("🎉 所有测试通过！Server 类重构成功！")
    else:
        print("❌ 部分测试失败，需要进一步检查")

    print("=" * 50)


if __name__ == "__main__":
    main()
