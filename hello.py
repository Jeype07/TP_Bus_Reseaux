from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import serial
import asyncio

app = FastAPI()

# Global variable to manage the serial connection
ser = None

class SerialConnectionRequest(BaseModel):
    port: str = "COM11"  # e.g., "COM3" on Windows or "/dev/ttyUSB0" on Linux
    baudrate: int = 115200  # Default baudrate

class Command(BaseModel):
    action: str

@app.post("/connect")
async def connect_to_serial(request: SerialConnectionRequest):
    global ser
    if ser and ser.is_open:
        return {"status": "Already connected", "port": ser.port}
    
    try:
        ser = serial.Serial(port=request.port, baudrate=request.baudrate, timeout=1)
        return {"status": "Connected", "port": ser.port, "baudrate": ser.baudrate}
    except serial.SerialException as e:
        raise HTTPException(status_code=400, detail=f"Failed to connect: {str(e)}")

@app.post("/disconnect")
async def disconnect_serial():
    global ser
    if ser and ser.is_open:
        ser.close()
        return {"status": "Disconnected"}
    return {"status": "No active connection"}

@app.post("/stm32/send-command/")
async def send_command(command: Command):
    """Envoie une commande au STM32."""
      
    # Envoi de la commande
    try:
        ser.write(command.action.encode('utf-8'))
        # Lecture de la réponse de la STM32
        await asyncio.sleep(0.1)  # Petit délai pour donner le temps à la STM32 de répondre
        response = ser.readline().decode('utf-8')
        cleaned_response = response.replace('\x00', '').strip()
        return {"status": "Command sent", "response": cleaned_response}

    except serial.SerialException as e:
        raise HTTPException(status_code=500, detail=f"Serial communication error: {e}")

# Stockage des températures, pressions et échelle
temperatures = [10,11]
pressures = []
scale = "K"  # Initialisation de l'échelle en Kelvin

# Modèle pour la température et la pression
class Temperature(BaseModel):
    temp: float 
    

class Pressure(BaseModel):
    pres: float

class ScaleChange(BaseModel):
    scale: str


# Créer une nouvelle température (POST /temp/)
@app.post("/temp/", status_code=201)
async def create_temperature(temp: Temperature):
    temperatures.append(temp.temp)
    return {"message": "Temperature added", "temperature": temp.temp}


# Créer une nouvelle pression (POST /pres/)
@app.post("/pres/", status_code=201)
async def create_pressure(pres: Pressure):
    pressures.append(pres.pres)
    return {"message": "Pressure added", "pressure": pres.pres}


# Récupérer toutes les températures précédentes (GET /temp/)
@app.get("/temp/", response_model=List[float])
async def get_temperatures():
    return temperatures


# Récupérer une température spécifique (GET /temp/{x})
@app.get("/temp/{x}")
async def get_temperature(x: int):
    if 0 <= x < len(temperatures):
        return {"temperature": temperatures[x]}
    else:
        raise HTTPException(status_code=404, detail="Temperature not found")


# 5. Récupérer toutes les pressions précédentes (GET /pres/)
@app.get("/pres/", response_model=List[float])
async def get_pressures():
    return pressures


# 6. Récupérer une pression spécifique (GET /pres/{x})
@app.get("/pres/{x}")
async def get_pressure(x: int):
    if 0 <= x < len(pressures):
        return {"pressure": pressures[x]}
    else:
        raise HTTPException(status_code=404, detail="Pressure not found")


# 7. Récupérer l'échelle (GET /scale/)
@app.get("/scale/")
async def get_scale():
    return {"scale": scale}


# 8. Calculer l'angle (temp * scale) (GET /angle/)
@app.get("/angle/")
async def get_angle():
    if temperatures:
        # Exemple de calcul, vous pouvez ajuster selon la logique
        angle = temperatures[-1] * (273 if scale == "K" else 1)
        return {"angle": angle}
    else:
        raise HTTPException(status_code=400, detail="No temperature available")


# 9. Changer l'échelle pour l'index {x} (POST /scale/{x})
@app.post("/scale/{x}")
async def change_scale(x: int, scale_change: ScaleChange):
    global scale
    if scale_change.scale not in ["K", "C", "F"]:
        raise HTTPException(status_code=400, detail="Invalid scale")
    scale = scale_change.scale
    return {"message": f"Scale changed to {scale}", "new_scale": scale}


# 10. Supprimer une température spécifique (DELETE /temp/{x})
@app.delete("/temp/{x}")
async def delete_temperature(x: int):
    if 0 <= x < len(temperatures):
        deleted_temp = temperatures.pop(x)
        return {"message": "Temperature deleted", "deleted_temperature": deleted_temp}
    else:
        raise HTTPException(status_code=404, detail="Temperature not found")


# 11. Supprimer une pression spécifique (DELETE /pres/{x})
@app.delete("/pres/{x}")
async def delete_pressure(x: int):
    if 0 <= x < len(pressures):
        deleted_pres = pressures.pop(x)
        return {"message": "Pressure deleted", "deleted_pressure": deleted_pres}
    else:
        raise HTTPException(status_code=404, detail="Pressure not found")

#uvicorn hello:app --host 192.168.88.231  launch serv on rasp pi

#uvicorn TP:app --host 0.0.0.0 --port 80 lauch serv on PC for network
