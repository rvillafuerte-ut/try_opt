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
    df = pd.read_csv(CSV_FILE)
    print(f"Datos cargados: {len(df)} filas")
    
    # ==== PLOT 1: Errores de posición y orientación ====
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    ax1.plot(df['timestamp'], df['pos_err'] * 1000, 'b-', linewidth=2)
    ax1.set_ylabel('Error de Posición (mm)', fontsize=12)
    ax1.set_xlabel('Tiempo (s)', fontsize=12)
    ax1.set_title('Error de Posición del End-Effector', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(df['timestamp'], np.degrees(df['rot_err']), 'r-', linewidth=2)
    ax2.set_ylabel('Error de Orientación (°)', fontsize=12)
    ax2.set_xlabel('Tiempo (s)', fontsize=12)
    ax2.set_title('Error de Orientación del End-Effector', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/errors.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/errors.png")
    plt.close()
    
    # ==== PLOT 2: Joint positions - q_cmd vs q_actual ====
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    joint_names = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']
    
    for i in range(6):
        axes[i].plot(df['timestamp'], np.degrees(df[f'q_cmd_{i}']), 'b-', label='q_cmd', linewidth=2)
        axes[i].plot(df['timestamp'], np.degrees(df[f'q_current_{i}']), 'r--', label='q_actual', linewidth=1.5, alpha=0.7)
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
    
    # ==== PLOT 3: Joint velocities (dq) ====
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    for i in range(6):
        axes[i].plot(df['timestamp'], np.degrees(df[f'dq_{i}']), 'g-', linewidth=2)
        axes[i].set_xlabel('Tiempo (s)', fontsize=10)
        axes[i].set_ylabel('Velocidad (°/s)', fontsize=10)
        axes[i].set_title(f'Joint {i+1}: {joint_names[i]}', fontsize=11, fontweight='bold')
        axes[i].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[i].grid(True, alpha=0.3)
    
    fig.suptitle('Velocidades de Articulaciones (dq = q_cmd - q_actual)', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/joint_velocities.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/joint_velocities.png")
    plt.close()
    
    # ==== PLOT 4: Trayectoria 3D de posición ====
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Trayectoria deseada
    ax.plot(df['p_des_x'], df['p_des_y'], df['p_des_z'], 
            'b-', linewidth=2, label='Deseada', alpha=0.7)
    
    # Trayectoria actual
    ax.plot(df['p_actual_x'], df['p_actual_y'], df['p_actual_z'], 
            'r--', linewidth=2, label='Actual', alpha=0.7)
    
    # Punto inicial
    ax.scatter(df['p_des_x'].iloc[0], df['p_des_y'].iloc[0], df['p_des_z'].iloc[0], 
               c='green', s=100, marker='o', label='Inicio')
    
    # Punto final
    ax.scatter(df['p_des_x'].iloc[-1], df['p_des_y'].iloc[-1], df['p_des_z'].iloc[-1], 
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
    
    # Estadísticas
    print("\n=== ESTADÍSTICAS ===")
    print(f"Error de posición medio: {df['pos_err'].mean()*1000:.2f} mm")
    print(f"Error de posición máximo: {df['pos_err'].max()*1000:.2f} mm")
    print(f"Error de orientación medio: {np.degrees(df['rot_err'].mean()):.2f}°")
    print(f"Error de orientación máximo: {np.degrees(df['rot_err'].max()):.2f}°")
    print("\n✓ Todas las gráficas generadas exitosamente!")

if __name__ == '__main__':
    main()
