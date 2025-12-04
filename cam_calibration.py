import cv2
import numpy as np
import os
import glob

# === CONFIGURAÇÕES DO TABULEIRO ===
chessboardSize = (9, 6)
square_size = 1.0

# Prepara pontos 3D do mundo real
objp = np.zeros((chessboardSize[0]*chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
objp *= square_size

# Onde serão salvas as imagens capturadas
save_dir = "captured_frames"
os.makedirs(save_dir, exist_ok=True)

print("=== MODO DE CAPTURA DE IMAGENS PARA CALIBRAÇÃO ===")
print("ESPAÇO = salvar imagem")
print("C = calibrar")
print("ESC = sair\n")

cap = cv2.VideoCapture(2)  # sua webcam
count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erro ao acessar a câmera.")
        break

    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1)

    if key == 27:  # ESC
        break

    elif key == 32:  # SPACE
        filename = os.path.join(save_dir, f"frame_{count}.jpg")
        cv2.imwrite(filename, frame)
        print(f"Imagem salva: {filename}")
        count += 1

    elif key == ord('c') or key == ord('C'):
        print("\n== INICIANDO CALIBRAÇÃO ==")
        cap.release()
        cv2.destroyAllWindows()
        break

cap.release()
cv2.destroyAllWindows()

# =====================================================================
# ========================= CALIBRAÇÃO ================================
# =====================================================================

images = glob.glob(f"{save_dir}/*.jpg")

if len(images) < 5:
    print("ERRO: Você precisa capturar pelo menos 5 imagens!")
    exit()

objpoints = []
imgpoints = []

print(f"Processando {len(images)} imagens...\n")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow("Detecção", img)
        cv2.waitKey(200)
    else:
        print(f"Tabuleiro não detectado na imagem: {fname}")

cv2.destroyAllWindows()

if len(objpoints) < 5:
    print("ERRO: Poucas imagens com tabuleiro detectado.")
    exit()

# Calibração
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n=== RESULTADOS DA CALIBRAÇÃO ===")
print("Matriz Intrínseca (cameraMatrix):\n", cameraMatrix)
print("\nCoeficientes de Distorção (distCoeffs):\n", distCoeffs)

# Salvar
np.save("cameraMatrix.npy", cameraMatrix)
np.save("distCoeffs.npy", distCoeffs)

print("\nArquivos salvos: cameraMatrix.npy e distCoeffs.npy")
print("Calibração concluída!")
