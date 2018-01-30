"""
Docs on how to do the markdown formatting:
http://docutils.sourceforge.net/docs/user/rst/quickref.html

Tool for previewing the markdown:
http://rst.ninjs.org/
"""

import os

from gym.scoreboard.registration import registry, add_task, add_group

# Discover API key from the environment. (You should never have to
# change api_base / web_base.)
'''api_key = os.environ.get('OPENAI_GYM_API_KEY')
api_base = os.environ.get('OPENAI_GYM_API_BASE', 'https://gym-api.openai.com')
web_base = os.environ.get('OPENAI_GYM_WEB_BASE', 'https://gym.openai.com')'''

# The following controls how various tasks appear on the
# scoreboard. These registrations can differ from what's registered in
# this repository.

# groups

add_group(
    id='gym-vehicle',
    name='gym-vehicle',
    description='TODO.'
)

add_task(
    id='GazeboCircuitLargeCatvehicleLidar-v0',
    group='gym-vehicle',
    summary='Test1.',
)

add_task(
    id='GazeboCircuitLargeCatvehicleLidarNn-v0',
    group='gym-vehicle',
    summary='Test2.',
)

registry.finalize()
