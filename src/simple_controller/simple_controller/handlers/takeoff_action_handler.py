import rclpy
import asyncio
import functools
from rclpy.action.server import GoalResponse, CancelResponse, ServerGoalHandle
from rclpy.lifecycle import LifecycleNode
from typing import Type, Any, Optional

from rclpy.task import Future

# Gerekli importlar
from .core_action_handler_base import ActionHandlerBase
from swarm_interfaces.action import Takeoff

# Sabitler
ALTITUDE_TOLERANCE = 0.25  # Metre cinsinden hedef irtifaya varış toleransı

class TakeoffActionHandler(ActionHandlerBase):
    """
    Takeoff aksiyonu için Action Handler.
    Bu versiyon, drone'a tek bir hedef pozisyon komutu göndererek çalışır
    ve hız kontrolü KULLANMAZ.
    """

    def __init__(self, parent_node: LifecycleNode, logger: Any):
        """
        TakeoffActionHandler'ı başlatır.
        """
        super().__init__(
            parent_node=parent_node,
            logger=logger,
            action_name="takeoff",
            action_type=Takeoff,
        )
        self._target_ned_pos: Optional[list[float]] = None
        self.execute_timer_period = 0.2 # Geri bildirim sıklığı (saniyede 5 kere)

    def goal_callback(self, goal_request: Takeoff.Goal) -> GoalResponse:
        """
        Gelen takeoff hedefini kabul veya reddeder.
        """
        # self.logger.debug(
        #     f"[{self.__class__.__name__}] Pozisyon kontrollü Takeoff hedef isteği alındı: {goal_request.target_height:.2f}m."
        # )
        if self.parent_node.get_current_local_position() is None:
            self.logger.error("Hedef reddedildi: Yerel pozisyon verisi henüz mevcut değil.")
            return GoalResponse.REJECT
        # self.logger.debug(f"Pozisyon kontrollü Takeoff hedefi kabul edildi.")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """
        Kabul edilen hedefin execute edilmesini başlatır.
        """
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        İptal isteklerini yönetir.
        """
        # self.logger.debug(f"[{self.__class__.__name__}] İptal isteği alındı.")
        # İptal durumunda drone'un mevcut pozisyonunda beklemesini (hover) sağlar.
        self.parent_node.command_hover_at_current_local_position()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Takeoff.Result:
        """
        Takeoff aksiyonunu execute eder: arm et, hedef pozisyonu belirle, durumu izle.
        """
        result = Takeoff.Result()
        request: Takeoff.Goal = goal_handle.request
        self.execute_future = Future()

        # --- 1. Hazırlık ve Kontroller ---
        current_pos = self.parent_node.get_current_local_position()
        if current_pos is None:
            result.success = False
            result.message = "Takeoff başlangıcında pozisyon alınamadı."
            goal_handle.abort()
            return result

        # === ANA DEĞİŞİKLİK: HEDEF POZİSYONUN HESAPLANMASI ===
        # Hedef pozisyon: mevcut X, Y ve istenen yeni Z (irtifa).
        # NED koordinat sisteminde Z ekseni aşağı doğru pozitif olduğu için
        # yükselmek, Z değerini azaltmak anlamına gelir.
        self._target_ned_pos = [
            current_pos.x,
            current_pos.y,
            current_pos.z - request.target_height
        ]
        takeoff_start_z = current_pos.z

        # --- 2. Offboard Moduna Geç ve Arm Et ---
        try:
            await self.parent_node.engage_offboard_mode()
            await self.parent_node.engage_arm_mode()
        except RuntimeError as e:
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        # === ANA DEĞİŞİKLİK: HEDEF POZİSYONUN TEK SEFERDE GÖNDERİLMESİ ===
        # self.logger.debug(f"Hedef pozisyon komutu gönderiliyor: NED [X={self._target_ned_pos[0]:.2f}, Y={self._target_ned_pos[1]:.2f}, Z={self._target_ned_pos[2]:.2f}]")
        self.parent_node.set_target_position_ned(
            self._target_ned_pos[0], self._target_ned_pos[1], self._target_ned_pos[2]
        )

        # --- 3. Geri Bildirim (Sadece İzleme) Döngüsünü Başlat ---
        partial_callback = functools.partial(self.execute_feedback_callback, Takeoff.Feedback(), goal_handle, takeoff_start_z)
        self.execute_feedback_timer = self.parent_node.create_timer(
            self.execute_timer_period, partial_callback, callback_group=self._callback_group
        )

        action_outcome = await self.execute_future

        # --- 4. Temizlik ve Sonucu İşleme ---
        if self.execute_feedback_timer:
            self.execute_feedback_timer.cancel()
        self._target_ned_pos = None

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Takeoff iptal edildi."
            goal_handle.canceled()
        elif action_outcome and action_outcome.get("success", False):
            result.success = True
            result.message = "Takeoff başarılı, hedef irtifaya ulaşıldı."
            result.final_height = action_outcome.get("final_height", 0.0)
            goal_handle.succeed()
        else:
            result.success = False
            result.message = "Takeoff başarısız oldu."
            goal_handle.abort()
        
        # self.logger.debug(result.message)
        return result

    async def execute_feedback_callback(self, feedback, goal_handle, takeoff_start_z):
        """
        Bu fonksiyon artık HIZ KOMUTU GÖNDERMEZ. Yalnızca drone'un hedefe
        ulaşıp ulaşmadığını izler.
        """
        if self._target_ned_pos is None:
            return

        if goal_handle.is_cancel_requested:
            if not self.execute_future.done():
                self.execute_future.set_result({"success": False})
            return

        current_position = self.parent_node.get_current_local_position()
        if current_position is None or not self.parent_node.is_armed():
            self.logger.error("Takeoff sırasında pozisyon kaybedildi veya drone disarm oldu!")
            self.parent_node.command_hover_at_current_local_position()
            if not self.execute_future.done():
                self.execute_future.set_result({"success": False})
            return

        # === ANA DEĞİŞİKLİK: ARTIK HIZ KOMUTU GÖNDERİLMEZ, SADECE İZLEME YAPILIR ===
        current_z_ned = current_position.z
        target_z_ned = self._target_ned_pos[2]
        
        # Tamamlanma kontrolü: Mevcut Z, hedef Z'ye yeterince yakın mı?
        if abs(current_z_ned - target_z_ned) < ALTITUDE_TOLERANCE:
            # self.logger.debug(f"Hedef irtifaya ulaşıldı. Pozisyonun korunması sağlanıyor.")
            # Hedefe varınca o pozisyonda sabit kalmasını garanti altına al.
            self.parent_node.set_target_position_ned(self._target_ned_pos[0], self._target_ned_pos[1], self._target_ned_pos[2])
            
            if not self.execute_future.done():
                final_height = takeoff_start_z - current_z_ned
                self.execute_future.set_result({"success": True, "final_height": final_height})
            return

        # Geri bildirim mesajını hesapla ve yayınla
        current_height_relative = takeoff_start_z - current_z_ned
        height_remaining = max(0.0, current_z_ned - target_z_ned) # Kalan mesafe
        feedback.current_height = current_height_relative
        feedback.height_remaining = height_remaining
        goal_handle.publish_feedback(feedback)
