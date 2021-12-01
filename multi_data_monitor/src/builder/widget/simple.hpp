// Copyright 2021 Takagi, Isamu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BUILDER__WIDGET__SIMPLE_HPP_
#define BUILDER__WIDGET__SIMPLE_HPP_

#include "../interface.hpp"

class QLabel;

namespace builder
{

class Simple : public Interface
{
public:
  using Interface::Interface;
  void Build(Dictionary & dict) override;
  void Callback(const YAML::Node & message) const override;

private:
  QLabel * label;
};

}  // namespace builder

#endif  // BUILDER__WIDGET__SIMPLE_HPP_
