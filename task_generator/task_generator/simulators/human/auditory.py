from __future__ import annotations

from task_generator_msgs.msg import SoundEvent
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
            SoundEvent,
            self._namespace(self.SOUND_EVENTS_TOPIC),
            10,
        )
        self._sound_commands_subscription = self.node.create_subscription(
            SoundEvent,
            self._namespace(self.SOUND_COMMANDS_TOPIC),
            self._cb_sound_command,
            10,
        )
        self._play_sound_service = self.node.create_service(
            Trigger,
            self.node.service_namespace(self.PLAY_SOUND_SERVICE),
            self._cb_play_sound,
        )

    def emit_sound_event(self, event: SoundEvent | str) -> None:
        if isinstance(event, SoundEvent):
            msg = event
        else:
            msg = SoundEvent()
            msg.header.stamp = self.node.sim_time.to_msg()
            msg.header.frame_id = "map"
            msg.event_id = f"manual:{msg.header.stamp.sec}:{msg.header.stamp.nanosec}"
            msg.source_agent_id = -1
            msg.source_agent_name = "manual"
            msg.sound_type = event.strip() or self.DEFAULT_SOUND_EVENT
            msg.label = msg.sound_type
            msg.asset_id = msg.sound_type
            msg.source_volume_db = 60.0
            msg.duration.sec = 1
            msg.loop = False

        self._sound_events_publisher.publish(msg)
        self._logger.info(f"auditory sound event: {msg.sound_type}")

    def _cb_sound_command(self, msg: SoundEvent) -> None:
        if not msg.sound_type.strip():
            msg.sound_type = self.DEFAULT_SOUND_EVENT
        if not msg.label.strip():
            msg.label = msg.sound_type
        if not msg.asset_id.strip():
            msg.asset_id = msg.sound_type

        self.emit_sound_event(msg)

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