import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.2


Window {
    title: qsTr("Full View")
    id: fullview
    readonly property int defaultWidth: 1600
    readonly property int defaultHeight: 900
    width: defaultWidth
    height: defaultHeight
    minimumWidth: 640
    minimumHeight: 360
    visible: true

    Shortcut {
        sequence: "Ctrl+F"
        onActivated: showFullScreen()
    }

    Shortcut {
        sequence: "Esc"
        onActivated: show()
    }

    Connections {
      target: multiview
      function onNewImage(topicName) {
          if (topicName == "diag_top_left_view") {
              one.reload(topicName);
          } else if (topicName == "top_view") {
              two.reload(topicName);
          } else if (topicName == "diag_top_right_view") {
              three.reload(topicName);
          } else if (topicName == "left_view") {
              four.reload(topicName);
          } else if (topicName == "front_view") {
              five.reload(topicName);
          } else if (topicName == "right_view") {
              six.reload(topicName);
          } else if (topicName == "diag_bottom_left_view") {
              seven.reload(topicName);
          } else if (topicName == "bottom_view") {
              eight.reload(topicName);
          } else if (topicName == "diag_bottom_right_view") {
              nine.reload(topicName);
          } else {
              return;
          }
      }
    }

    GridLayout {
        anchors.fill: parent
        columns: 3
        rows: 3
        columnSpacing: 0
        rowSpacing: 0

        Image {
            id: one
            fillMode: Image.PreserveAspectFit
            Layout.column: 0
            Layout.row: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://diag_top_left_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: two
            fillMode: Image.PreserveAspectFit
            Layout.column: 1
            Layout.row: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://top_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: three
            fillMode: Image.PreserveAspectFit
            Layout.column: 2
            Layout.row: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://diag_top_right_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: four
            fillMode: Image.PreserveAspectFit
            Layout.column: 0
            Layout.row: 1
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://left_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: five
            fillMode: Image.PreserveAspectFit
            Layout.column: 1
            Layout.row: 1
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://front_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: six
            fillMode: Image.PreserveAspectFit
            Layout.column: 2
            Layout.row: 1
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://right_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: seven
            fillMode: Image.PreserveAspectFit
            Layout.column: 0
            Layout.row: 2
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://diag_bottom_left_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: eight
            fillMode: Image.PreserveAspectFit
            Layout.column: 1
            Layout.row: 2
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://bottom_view/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: nine
            fillMode: Image.PreserveAspectFit
            Layout.column: 2
            Layout.row: 2
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload() {
              source = "image://diag_bottom_right_view/" + Math.random().toString(36).substr(2, 5);
            }
        }
    }
}