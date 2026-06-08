from __future__ import annotations

from std_msgs.msg import String
from std_srvs.srv import Trigger

from task_generator.simulators.human.dummy import DummyHumanSimulator


class AuditoryHumanSimulator(DummyHumanSimulator):
    """Human simulator adapter that exposes a sound-event interface.

    It delegates all spawn/remove behavior to DummyHumanSimulator and adds:
    - input topic: human_sound_commands
    - output topic: human_sound_events
    - service: play_sound
    """

    SOUND_COMMANDS_TOPIC = "human_sound_commands"
    SOUND_EVENTS_TOPIC = "human_sound_events"
    PLAY_SOUND_SERVICE = "play_sound"
    DEFAULT_SOUND_EVENT = "sound"

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

        self._sound_events_publisher = self.node.create_publisher(
            String,
            self._namespace(self.SOUND_EVENTS_TOPIC),
            10,
        )
        self._sound_commands_subscription = self.node.create_subscription(
            String,
            self._namespace(self.SOUND_COMMANDS_TOPIC),
            self._cb_sound_command,
            10,
        )
        self._play_sound_service = self.node.create_service(
            Trigger,
            self.node.service_namespace(self.PLAY_SOUND_SERVICE),
            self._cb_play_sound,
        )

    def emit_sound_event(self, event: str) -> None:
        msg = String()
        msg.data = event
        self._sound_events_publisher.publish(msg)
        self._logger.info(f"auditory sound event: {event}")

    def _cb_sound_command(self, msg: String) -> None:
        event = msg.data.strip() or self.DEFAULT_SOUND_EVENT
        self.emit_sound_event(event)

    def _cb_play_sound(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        self.emit_sound_event(self.DEFAULT_SOUND_EVENT)
        response.success = True
        response.message = self.DEFAULT_SOUND_EVENT
        return response