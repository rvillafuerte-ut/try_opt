#!/usr/bin/env python3
"""
Script para generar gráficas de datos del controlador de teleoperación.
Genera 4 plots: errores de posición/orientación, joints commanded vs actual, y velocidades.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path

# Configuración
CSV_FILE = "/home/utec/try_opt/teleop_data.csv"
PLOTS_DIR = "/home/utec/try_opt/plots"

def main():
    # Crear directorio de plots si no existe
    Path(PLOTS_DIR).mkdir(parents=True, exist_ok=True)
    
    # Leer datos
    print(f"Leyendo datos de {CSV_FILE}...")
    try:
        df = pd.read_csv(CSV_FILE)
        print(f"Datos cargados: {len(df)} filas")
    except Exception as e:
        print(f"Error leyendo CSV: {e}")
        return

    # Calcular errores cartesianos
    df['pos_err'] = np.sqrt((df['pd0'] - df['pa0'])**2 + 
                            (df['pd1'] - df['pa1'])**2 + 
                            (df['pd2'] - df['pa2'])**2)
    
    # ==== PLOT 1: Error de posición Cartesiano ====
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    ax1.plot(df['time'], df['pos_err'] * 1000, 'b-', linewidth=2)
    ax1.set_ylabel('Error de Posición (mm)', fontsize=12)
    ax1.set_xlabel('Tiempo (s)', fontsize=12)
    ax1.set_title('Error de Seguimiento Cartesiano', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/cartesian_error.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/cartesian_error.png")
    plt.close()
    
    # ==== PLOT 2: Joint positions - q_cmd vs q_actual ====
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    joint_names = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']
    
    for i in range(6):
        axes[i].plot(df['time'], np.degrees(df[f'qc{i}']), 'b-', label='q_cmd', linewidth=2)
        axes[i].plot(df['time'], np.degrees(df[f'qa{i}']), 'r--', label='q_actual', linewidth=1.5, alpha=0.7)
        axes[i].set_xlabel('Tiempo (s)', fontsize=10)
        axes[i].set_ylabel('Ángulo (°)', fontsize=10)
        axes[i].set_title(f'Joint {i+1}: {joint_names[i]}', fontsize=11, fontweight='bold')
        axes[i].legend(loc='upper right', fontsize=9)
        axes[i].grid(True, alpha=0.3)
    
    fig.suptitle('Posiciones de Articulaciones: Comandadas vs Actuales', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/joint_positions.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/joint_positions.png")
    plt.close()
    
    # ==== PLOT 3: Joint velocities (dq actual) ====
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    for i in range(6):
        axes[i].plot(df['time'], np.degrees(df[f'dqa{i}']), 'g-', linewidth=2)
        axes[i].set_xlabel('Tiempo (s)', fontsize=10)
        axes[i].set_ylabel('Velocidad (°/s)', fontsize=10)
        axes[i].set_title(f'Joint {i+1}: {joint_names[i]} (Actual)', fontsize=11, fontweight='bold')
        axes[i].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[i].grid(True, alpha=0.3)
    
    fig.suptitle('Velocidades Reales de Articulaciones', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/joint_velocities.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/joint_velocities.png")
    plt.close()
    
    # ==== PLOT 4: Trayectoria 3D de posición ====
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Trayectoria deseada
    ax.plot(df['pd0'], df['pd1'], df['pd2'], 
            'b-', linewidth=2, label='Deseada', alpha=0.7)
    
    # Trayectoria actual
    ax.plot(df['pa0'], df['pa1'], df['pa2'], 
            'r--', linewidth=2, label='Actual', alpha=0.7)
    
    # Punto inicial
    ax.scatter(df['pd0'].iloc[0], df['pd1'].iloc[0], df['pd2'].iloc[0], 
               c='green', s=100, marker='o', label='Inicio')
    
    # Punto final
    ax.scatter(df['pd0'].iloc[-1], df['pd1'].iloc[-1], df['pd2'].iloc[-1], 
               c='red', s=100, marker='x', label='Final')
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('Trayectoria 3D del End-Effector', fontsize=14, fontweight='bold')
    ax.legend(loc='upper left', fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/trajectory_3d.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/trajectory_3d.png")
    plt.close()
    
    print("\n✓ Todas las gráficas generadas exitosamente!")

if __name__ == '__main__':
    main()
