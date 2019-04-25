# -*- coding: future_fstrings -*-

"""Wrapper class for QSettings which implements subscriptions.

	Observe a key with a callback, called when the key is
	changed or initialized.
"""

import json

from PyQt5.QtCore import QSettings, QCoreApplication
_app = QCoreApplication([]) #Required, or nothing will get saved.
_settings = QSettings('Krontech', 'dbus control api') #in ~/.config/Krontech/back-of-camera interface.conf

def set(key: str, value: any) -> None:
	"""See http://doc.qt.io/qt-5/qsettings.html#setValue"""
	
	#Do some typechecking, this gets confusing because the value is cast to a string (according to non-python rules) only after the app is restarted.
	if not isinstance(key, str):
		raise TypeError(f'settings.setValue(key, value) only accepts str keys, because that\'s what the underlying store accepts. It got passed the key {key}, a {type(key)}.')
	
	_settings.setValue(key, json.dumps(value))


def get(key: str, default: any = None) -> any:
	"""See http://doc.qt.io/qt-5/qsettings.html#value"""
	return json.loads(_settings.value(key, json.dumps(default)))