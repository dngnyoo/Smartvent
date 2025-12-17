import pygame
import sys

# 디버깅을 위한 로그 출력
print(">>> [1단계] Pygame 모듈 초기화 중...")
pygame.init()

# 전체 화면 대신 작은 창모드로 실행 (호환성 문제 최소화)
print(">>> [2단계] 640x480 윈도우 생성 시도 중...")
try:
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("라즈베리파이 테스트")
    print(">>> [성공] 윈도우 생성 완료! 화면을 확인하세요.")
except Exception as e:
    print(f">>> [에러] 윈도우 생성 실패: {e}")
    sys.exit()

clock = pygame.time.Clock()
x_pos = 0
speed = 5

# 메인 루프
print(">>> [3단계] 애니메이션 루프 진입")
while True:
    # 1. 이벤트 처리 (창 닫기 버튼)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print(">>> 종료 요청 받음")
            pygame.quit()
            sys.exit()

    # 2. 로직 (좌우 왕복 이동)
    x_pos += speed
    if x_pos > 590 or x_pos < 0: # 벽에 닿으면 반대 방향으로
        speed = speed * -1

    # 3. 그리기
    screen.fill((0, 0, 0))  # 검은색 배경
    pygame.draw.rect(screen, (255, 0, 0), (x_pos, 200, 50, 50))  # 빨간색 사각형

    # 4. 화면 업데이트
    pygame.display.flip()
    
    # 60 FPS 제한
    clock.tick(60)