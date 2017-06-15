/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../SBMT-0.0.1/include/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[19];
    char stringdata0[449];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 40), // "on_calibrationParamsBrowseBut..."
QT_MOC_LITERAL(2, 52, 0), // ""
QT_MOC_LITERAL(3, 53, 32), // "on_leftImageBrowseButton_clicked"
QT_MOC_LITERAL(4, 86, 33), // "on_rightImageBrowseButton_cli..."
QT_MOC_LITERAL(5, 120, 24), // "on_computeButton_clicked"
QT_MOC_LITERAL(6, 145, 23), // "on_exportButton_clicked"
QT_MOC_LITERAL(7, 169, 22), // "on_image480PRB_clicked"
QT_MOC_LITERAL(8, 192, 22), // "on_image720PRB_clicked"
QT_MOC_LITERAL(9, 215, 23), // "on_image1080PRB_clicked"
QT_MOC_LITERAL(10, 239, 20), // "on_image2KRB_clicked"
QT_MOC_LITERAL(11, 260, 20), // "on_image4KRB_clicked"
QT_MOC_LITERAL(12, 281, 20), // "on_sgbmRadio_clicked"
QT_MOC_LITERAL(13, 302, 19), // "on_sbmRadio_clicked"
QT_MOC_LITERAL(14, 322, 23), // "on_actionHelp_triggered"
QT_MOC_LITERAL(15, 346, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(16, 371, 23), // "on_actionExit_triggered"
QT_MOC_LITERAL(17, 395, 27), // "on_actionGenerate_triggered"
QT_MOC_LITERAL(18, 423, 25) // "on_actionExport_triggered"

    },
    "MainWindow\0on_calibrationParamsBrowseButton_clicked\0"
    "\0on_leftImageBrowseButton_clicked\0"
    "on_rightImageBrowseButton_clicked\0"
    "on_computeButton_clicked\0"
    "on_exportButton_clicked\0on_image480PRB_clicked\0"
    "on_image720PRB_clicked\0on_image1080PRB_clicked\0"
    "on_image2KRB_clicked\0on_image4KRB_clicked\0"
    "on_sgbmRadio_clicked\0on_sbmRadio_clicked\0"
    "on_actionHelp_triggered\0"
    "on_actionAbout_triggered\0"
    "on_actionExit_triggered\0"
    "on_actionGenerate_triggered\0"
    "on_actionExport_triggered"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   99,    2, 0x08 /* Private */,
       3,    0,  100,    2, 0x08 /* Private */,
       4,    0,  101,    2, 0x08 /* Private */,
       5,    0,  102,    2, 0x08 /* Private */,
       6,    0,  103,    2, 0x08 /* Private */,
       7,    0,  104,    2, 0x08 /* Private */,
       8,    0,  105,    2, 0x08 /* Private */,
       9,    0,  106,    2, 0x08 /* Private */,
      10,    0,  107,    2, 0x08 /* Private */,
      11,    0,  108,    2, 0x08 /* Private */,
      12,    0,  109,    2, 0x08 /* Private */,
      13,    0,  110,    2, 0x08 /* Private */,
      14,    0,  111,    2, 0x08 /* Private */,
      15,    0,  112,    2, 0x08 /* Private */,
      16,    0,  113,    2, 0x08 /* Private */,
      17,    0,  114,    2, 0x08 /* Private */,
      18,    0,  115,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_calibrationParamsBrowseButton_clicked(); break;
        case 1: _t->on_leftImageBrowseButton_clicked(); break;
        case 2: _t->on_rightImageBrowseButton_clicked(); break;
        case 3: _t->on_computeButton_clicked(); break;
        case 4: _t->on_exportButton_clicked(); break;
        case 5: _t->on_image480PRB_clicked(); break;
        case 6: _t->on_image720PRB_clicked(); break;
        case 7: _t->on_image1080PRB_clicked(); break;
        case 8: _t->on_image2KRB_clicked(); break;
        case 9: _t->on_image4KRB_clicked(); break;
        case 10: _t->on_sgbmRadio_clicked(); break;
        case 11: _t->on_sbmRadio_clicked(); break;
        case 12: _t->on_actionHelp_triggered(); break;
        case 13: _t->on_actionAbout_triggered(); break;
        case 14: _t->on_actionExit_triggered(); break;
        case 15: _t->on_actionGenerate_triggered(); break;
        case 16: _t->on_actionExport_triggered(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
