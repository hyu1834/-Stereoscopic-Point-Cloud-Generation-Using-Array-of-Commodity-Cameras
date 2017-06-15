/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[14];
    char stringdata0[370];
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
QT_MOC_LITERAL(3, 53, 36), // "on_blockMatchingBrowseButton_..."
QT_MOC_LITERAL(4, 90, 32), // "on_leftImageBrowseButton_clicked"
QT_MOC_LITERAL(5, 123, 33), // "on_rightImageBrowseButton_cli..."
QT_MOC_LITERAL(6, 157, 31), // "on_resultPCBrowseButton_clicked"
QT_MOC_LITERAL(7, 189, 27), // "on_generatePCButton_clicked"
QT_MOC_LITERAL(8, 217, 25), // "on_exportPCButton_clicked"
QT_MOC_LITERAL(9, 243, 27), // "on_actionGenerate_triggered"
QT_MOC_LITERAL(10, 271, 25), // "on_actionExport_triggered"
QT_MOC_LITERAL(11, 297, 23), // "on_actionExit_triggered"
QT_MOC_LITERAL(12, 321, 23), // "on_actionHelp_triggered"
QT_MOC_LITERAL(13, 345, 24) // "on_actionAbout_triggered"

    },
    "MainWindow\0on_calibrationParamsBrowseButton_clicked\0"
    "\0on_blockMatchingBrowseButton_clicked\0"
    "on_leftImageBrowseButton_clicked\0"
    "on_rightImageBrowseButton_clicked\0"
    "on_resultPCBrowseButton_clicked\0"
    "on_generatePCButton_clicked\0"
    "on_exportPCButton_clicked\0"
    "on_actionGenerate_triggered\0"
    "on_actionExport_triggered\0"
    "on_actionExit_triggered\0on_actionHelp_triggered\0"
    "on_actionAbout_triggered"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    0,   85,    2, 0x08 /* Private */,

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

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_calibrationParamsBrowseButton_clicked(); break;
        case 1: _t->on_blockMatchingBrowseButton_clicked(); break;
        case 2: _t->on_leftImageBrowseButton_clicked(); break;
        case 3: _t->on_rightImageBrowseButton_clicked(); break;
        case 4: _t->on_resultPCBrowseButton_clicked(); break;
        case 5: _t->on_generatePCButton_clicked(); break;
        case 6: _t->on_exportPCButton_clicked(); break;
        case 7: _t->on_actionGenerate_triggered(); break;
        case 8: _t->on_actionExport_triggered(); break;
        case 9: _t->on_actionExit_triggered(); break;
        case 10: _t->on_actionHelp_triggered(); break;
        case 11: _t->on_actionAbout_triggered(); break;
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
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
