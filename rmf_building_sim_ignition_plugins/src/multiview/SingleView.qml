import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.2


Window {
    title: qsTr(singleViewWindow.cameraView)
    id: singleViewWindow
    x: 0
    y: 0
    readonly property int defaultWidth: 7680
    readonly property int defaultHeight: 2160
    width: defaultWidth
    height: defaultHeight
    minimumWidth: 768
    minimumHeight: 216
    visible: true
    flags: Qt.Window
    property string cameraView: ""

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
        cameraView.reload();
      }
    }

    Image {
      id: cameraView
      fillMode: Image.PreserveAspectFit
      anchors.fill: parent
      function reload() {
        source = "image://" + singleViewWindow.cameraView + "/" + Math.random().toString(36).substr(2, 5);
      }
    }
}