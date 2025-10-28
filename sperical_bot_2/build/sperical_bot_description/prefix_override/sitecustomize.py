import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/omkolse/DHDR_bot/sperical_bot_2/install/sperical_bot_description'
