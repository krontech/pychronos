.. pychronos documentation master file, created by
   sphinx-quickstart on Fri May 10 22:19:11 2019.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. title::
	Index

:mod:`pychronos` --- Chronos Camera Control
===========================================

The :mod:`pychronos` module provides programmatic high-level control over a Chronos Camera. To do so, it exposes a D-Bus API for other software to consume. This document covers the PyChronos project and how to use it.

.. contents:: Table of Contents
	:depth: 2	


History
-------
In the beginning, there was camApp. And it was glorious, and ran all parts of the camera from the UI to the FPGA, and the engineers were happy.


.. Include the SVG image inline, it's got some links in it we want to work. Otherwise, include it as an image because it'll still be better than nothing, and the image will fall back gracefully to the alt text provided for non-html formats.
.. raw:: html
	:file: _static/previous-architecture.svg
.. only:: not html

	.. image:: _static/previous-architecture.svg
		:align: center
		:alt: Communicating directly with the hardware, two daemons expose video (in chronos-cli) and control (pychronos) functionality over a D-Bus interface. Consuming those interfaces, there are the back-of-camera user interface (chronos-gui-2) and web interface bridge (in chronos-web-interface). Connected to the web interface bridge are any remote clients.


As time passed, the engineers grew numerous and discontentment spread; and there was conflict, and confusion; and it was declared that camApp be split into several components each with a clearly-defined responsibility. And it was declared those components would be as follows:

.. raw:: html
	:file: _static/current-architecture.svg
.. only:: not html

	.. image:: _static/current-architecture.svg
		:align: center
		:alt: Communicating directly with the hardware, two daemons expose video (in chronos-cli) and control (pychronos) functionality over a D-Bus interface. Consuming those interfaces, there are the back-of-camera user interface (chronos-gui-2) and web interface bridge (in chronos-web-interface). Connected to the web interface bridge are any remote clients.

Control API
^^^^^^^^^^^
This project. Configures the camera and, internally, coordinates bring-up of the camera hardware with the the Video API. Partially wraps the Video API as a result. Exposes functionality over D-Bus.

Video API
^^^^^^^^^
Deals with video recording and display. Starts and stops the stream, and internally routes the picture to the the HDMI port and the back-of-camera display. Exposes functionality over D-Bus.

User Interface
^^^^^^^^^^^^^^
Synchronise your brain with a camera! The UI displays the current camera state and lets people change it how they want. It translates human input into D-Bus calls to the Video and Control APIs, as well as listens to the D-Bus APIs for updates to display.

Web Interface
^^^^^^^^^^^^^
Synchronise your network client with a camera! The Web Interface exposes the Control and Video D-Bus APIs over HTTP so a remote client can make use of them. It must be enabled in the Network Settings screen on the camera before it will start.

Your Interface Here
^^^^^^^^^^^^^^^^^^^
Write your own on-board software. This can be as simple as dropping some new HTML files into the web directory the current app is running out of, to enable new, convenient functionality for your own project, or as complex as writing a complete replacement for the back-of-camera user interface.

Web App
^^^^^^^
After enabling the web interface, point a web browser at your camera for a remote-control app. (Well, soon, at any rate. This component is not complete yet!)

Your Client Here
^^^^^^^^^^^^^^^^
Write your own remote client! Control the camera using a script, program, or web page on another computer or mobile device. (This is generally easier than writing your own on-board interface.)







PyChronos Components
--------------------

.. toctree::
	:maxdepth: 2
	:caption: Subcomponents
	:name: mastertoc

	camera
	sensor

Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

