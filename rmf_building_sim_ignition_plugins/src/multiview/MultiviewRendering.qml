import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.2


Window {
    title: qsTr("Multi-view display")
    id: twoByTwo
    readonly property int defaultWidth: 1600
    readonly property int defaultHeight: 900
    width: defaultWidth
    height: defaultHeight
    minimumWidth: 640
    minimumHeight: 360
    visible: true
    property string topLeftView: ""
    property string bottomLeftView: ""
    property string topRightView: ""
    property string bottomRightView: ""

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
          if (topicName == twoByTwo.topLeftView) {
              topLeftImage.reload(topicName);
          } else if (topicName == twoByTwo.bottomLeftView) {
              bottomLeftImage.reload(topicName);
          } else if (topicName == twoByTwo.topRightView) {
              topRightImage.reload(topicName);
          } else if (topicName == twoByTwo.bottomRightView) {
              bottomRightImage.reload(topicName);
          } else {
              return;
          }
      }
    }

    GridLayout {
        anchors.fill: parent
        columns: 2
        rows: 2
        columnSpacing: 0
        rowSpacing: 0

        Image {
            id: topLeftImage
            fillMode: Image.PreserveAspectFit
            Layout.column: 0
            Layout.row: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload(topicName) {
                source = "image://" + topicName + "/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: bottomLeftImage
            fillMode: Image.PreserveAspectFit
            Layout.column: 0
            Layout.row: 1
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload(topicName) {
                source = "image://" + topicName + "/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: topRightImage
            fillMode: Image.PreserveAspectFit
            Layout.column: 1
            Layout.row: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload(topicName) {
                source = "image://" + topicName + "/" + Math.random().toString(36).substr(2, 5);
            }
        }

        Image {
            id: bottomRightImage
            fillMode: Image.PreserveAspectFit
            Layout.column: 1
            Layout.row: 1
            Layout.fillWidth: true
            Layout.fillHeight: true
            function reload(topicName) {
                source = "image://" + topicName + "/" + Math.random().toString(36).substr(2, 5);
            }
        }
    }
}