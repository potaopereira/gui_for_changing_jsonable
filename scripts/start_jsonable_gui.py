#!/usr/bin/env python

import sys

# from gui.tabbedGUI import tabbedGUIPlugin
from rqt_gui.main import Main

plugin = 'ChooseSequenceOfJsonables'
# plugin = 'Jsonable'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))


