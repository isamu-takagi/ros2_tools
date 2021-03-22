// Copyright 2021 Takagi Isamu
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

#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include "window.hpp"

class RosApplication final
{
    public:

        RosApplication(int argc, char ** argv)
        {
            rclcpp::init(argc, argv);
        }

        ~RosApplication()
        {
            rclcpp::shutdown();
        }

        // rclcpp::spin(std::make_shared<Node>()); or spin_some
};

int main(int argc, char ** argv)
{
    QApplication application(argc, argv);
    application.setOrganizationName("tkgism");
    application.setApplicationName("sample");

    RosApplication ros(argc, argv);
    MyWindow window(std::make_shared<rclcpp::Node>("sample"));
    window.show();
    return application.exec();
}
