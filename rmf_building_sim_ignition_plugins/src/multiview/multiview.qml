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
import QtQuick.Controls.Material 2.1

ToolBar {
    Layout.minimumWidth: 280
    Layout.minimumHeight: 370

    id: main
    background: Rectangle {
        color: "transparent"
    }

    property int tooltipDelay: 500
    property int tooptipTimeout: 1000

    ColumnLayout {
        id: mainColumn
        spacing: 10

        ColumnLayout {
            id: singleViewColumn
            spacing: 5

            RowLayout {

                // Padding for title
                Rectangle {
                    width: 5
                    height: 40
                    color: "transparent"
                }

                Label {
                    text: "Single View"
                    horizontalAlignment: Qt.AlignHLeft
                    verticalAlignment: Qt.AlignVCenter
                    Layout.fillWidth: true
                }

                Button {
                    id: singleView
                    text: qsTr("Open")

                    onClicked: {
                        var component = Qt.createComponent("SingleView.qml")
                        if (component.status != Component.Ready) {
                            if (component.status == Component.Error) {
                                console.debug("Error: " + component.errorString());
                                return;
                            }
                        }
                        var multiview = component.createObject(main, {"cameraView": combo.currentText});
                        multiview.show();
                    }
                }
            }

            ComboBox {
                id: combo
                Layout.column: 0
                Layout.row: 0
                Layout.fillWidth: true
                model: multiview.topicList
                ToolTip.visible: hovered
                ToolTip.delay: main.tooltipDelay
                ToolTip.timeout: main.tooltipTimeout
                ToolTip.text: qsTr("List of topics streaming camera images")
            }
        }

        ColumnLayout {
            id: twoByTwoColumn
            spacing: 5

            RowLayout {

                // Padding for title
                Rectangle {
                    width: 5
                    height: 40
                    color: "transparent"
                }

                Label {
                    text: "Multiview (2x2)"
                    horizontalAlignment: Qt.AlignHLeft
                    verticalAlignment: Qt.AlignVCenter
                    Layout.fillWidth: true
                }

                Button {
                    id: twoByTwoButton
                    text: qsTr("Open")

                    onClicked: {
                        var component = Qt.createComponent("MultiviewRendering.qml")
                        if (component.status != Component.Ready) {
                            if (component.status == Component.Error) {
                                console.debug("Error: " + component.errorString());
                                return;
                            }
                        }
                        var multiview = component.createObject(main, {"topLeftView": topLeftCombo.currentText,
                                                                    "bottomLeftView": bottomLeftCombo.currentText,
                                                                    "topRightView": topRightCombo.currentText,
                                                                    "bottomRightView": bottomRightCombo.currentText});
                        multiview.show();
                    }
                }
            }

            GridLayout {
                columns: 2
                rows: 3
                columnSpacing: 5

                ComboBox {
                    id: topLeftCombo
                    Layout.column: 0
                    Layout.row: 0
                    Layout.fillWidth: true
                    model: multiview.topicList
                    ToolTip.visible: hovered
                    ToolTip.delay: main.tooltipDelay
                    ToolTip.timeout: main.tooltipTimeout
                    ToolTip.text: qsTr(topLeftCombo.currentText)
                }

                ComboBox {
                    id: bottomLeftCombo
                    Layout.column: 0
                    Layout.row: 1
                    Layout.fillWidth: true
                    model: multiview.topicList
                    ToolTip.visible: hovered
                    ToolTip.delay: main.tooltipDelay
                    ToolTip.timeout: main.tooltipTimeout
                    ToolTip.text: qsTr(bottomLeftCombo.currentText)
                }

                ComboBox {
                    id: topRightCombo
                    Layout.column: 1
                    Layout.row: 0
                    Layout.fillWidth: true
                    model: multiview.topicList
                    ToolTip.visible: hovered
                    ToolTip.delay: main.tooltipDelay
                    ToolTip.timeout: main.tooltipTimeout
                    ToolTip.text: qsTr(topRightCombo.currentText)
                }

                ComboBox {
                    id: bottomRightCombo
                    Layout.column: 1
                    Layout.row: 1
                    Layout.fillWidth: true
                    model: multiview.topicList
                    ToolTip.visible: hovered
                    ToolTip.delay: main.tooltipDelay
                    ToolTip.timeout: main.tooltipTimeout
                    ToolTip.text: qsTr(bottomRightCombo.currentText)
                }
            }
        }

        RowLayout {

            // Padding for title
            Rectangle {
                width: 5
                height: 40
                color: "transparent"
            }

            Label {
                text: "Full View (3x3)"
                horizontalAlignment: Qt.AlignHLeft
                verticalAlignment: Qt.AlignVCenter
                Layout.fillWidth: true
            }

            Button {
                id: fullView
                text: qsTr("Open")

                onClicked: {
                    var component = Qt.createComponent("FullView.qml")
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
        
        ScrollBar {
            id: scroll
            hoverEnabled: true
            active: hovered || pressed
            orientation: Qt.Vertical
            size: 2
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.bottom: parent.bottom
        }
    }
}