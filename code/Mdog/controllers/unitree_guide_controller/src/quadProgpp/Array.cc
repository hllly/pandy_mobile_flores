// $Id: Array.cc 201 2008-05-18 19:47:38Z digasper $
// 本文件属于 QuadProg++：
// Copyright (C) 2006--2009 Luca Di Gaspero.
//
// 本软件可依据 MIT 许可证的条款进行修改与分发，详情参阅 LICENSE 文件。

#include "Array.hh"

/**
  索引工具函数
 */

namespace quadprogpp {

std::set<unsigned int> seq(unsigned int s, unsigned int e) {
  std::set<unsigned int> tmp;
  for (unsigned int i = s; i <= e; i++) tmp.insert(i);

  return tmp;
}

std::set<unsigned int> singleton(unsigned int i) {
  std::set<unsigned int> tmp;
  tmp.insert(i);

  return tmp;
}

}  // 命名空间 quadprogpp
