/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.2

ToolBar {
    Layout.minimumWidth: 280
    Layout.minimumHeight: 370

    id: main
    background: Rectangle {
        color: "transparent"
    }

    GridLayout {
        anchors.fill: parent
        columns: 2
        rows: 3
        columnSpacing: 5

        Button {
            text: qsTr("2x2 Multiview")
            Layout.column: 0
            Layout.row: 0

            onClicked: {
                var component = Qt.createComponent("MultiviewRendering.qml")
                if (component.status != Component.Ready) {
                    if (component.status == Component.Error) {
                        console.debug("Error: " + component.errorString());
                        return;
                    }
                }
                var multiview = component.createObject(main);
                multiview.show();
            }
        }

        Button {
            text: qsTr("1-3 Multiview")
            Layout.column: 1
            Layout.row: 0

            onClicked: {
                var component = Qt.createComponent("MultiviewRenderingTest.qml")
                if (component.status != Component.Ready) {
                    if (component.status == Component.Error) {
                        console.debug("Error: " + component.errorString());
                        return;
                    }
                }
                var multiviewTest = component.createObject(main);
                multiviewTest.show();
            }
        }


        Button {
            text: qsTr("Front view")
            Layout.column: 0
            Layout.row: 1

            onClicked: {
                var component = Qt.createComponent("FrontView.qml")
                if (component.status != Component.Ready) {
                    if (component.status == Component.Error) {
                        console.debug("Error: " + component.errorString());
                        return;
                    }
                }
                var multiview = component.createObject(main);
                multiview.show();
            }
        }

        Button {
            text: qsTr("Left view")
            Layout.column: 1
            Layout.row: 1

            onClicked: {
                var component = Qt.createComponent("LeftView.qml")
                if (component.status != Component.Ready) {
                    if (component.status == Component.Error) {
                        console.debug("Error: " + component.errorString());
                        return;
                    }
                }
                var multiview = component.createObject(main);
                multiview.show();
            }
        }

        Button {
            text: qsTr("Right view")
            Layout.column: 2
            Layout.row: 0

            onClicked: {
                var component = Qt.createComponent("RightView.qml")
                if (component.status != Component.Ready) {
                    if (component.status == Component.Error) {
                        console.debug("Error: " + component.errorString());
                        return;
                    }
                }
                var multiview = component.createObject(main);
                multiview.show();
            }
        }
    }
}