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
        
        # Estadísticas de velocidad
        vel_max = df['vel_max_req'].max()
        vel_mean = df['vel_max_req'].mean()
        n_saturated = (df['saturated'] == 1).sum()
        pct_saturated = n_saturated / len(df) * 100
        
        print(f"\n Estadísticas de Velocidad:")
        print(f"  Vel máxima requerida: {vel_max:.2f} rad/s")
        print(f"  Vel promedio: {vel_mean:.2f} rad/s")
        print(f"  Puntos saturados: {n_saturated} ({pct_saturated:.1f}%)")
        if vel_max > 3.5:
            print(f"  ADVERTENCIA: Pico peligroso detectado (>{vel_max:.2f} rad/s)")
        
    except Exception as e:
        print(f"Error leyendo CSV: {e}")
        return

    # Calcular error articular (q_cmd - q_actual)
    for i in range(6):
        df[f'e_q{i}'] = df[f'qc{i}'] - df[f'qa{i}']
    df['error_q_norm'] = np.sqrt(sum(df[f'e_q{i}']**2 for i in range(6)))
    
    # ==== PLOT 1: Velocidades Requeridas y Control Adaptativo ====
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Subplot 1: Velocidades requeridas por articulación
    for i in range(6):
        axes[0].plot(df['time'], np.abs(df[f'vel_req{i}']), label=f'Joint {i+1}', linewidth=1.5, alpha=0.7)
    axes[0].plot(df['time'], df['vel_max_req'], 'k-', label='Vel Max Requerida', linewidth=2.5)
    
    # Marcar zonas saturadas en rojo
    saturated_mask = df['saturated'] == 1
    if saturated_mask.any():
        axes[0].scatter(df.loc[saturated_mask, 'time'], df.loc[saturated_mask, 'vel_max_req'], 
                       c='red', s=50, marker='x', label='SATURADO', zorder=5)
        n_saturated = saturated_mask.sum()
        axes[0].text(0.02, 0.98, f'⚠ {n_saturated} puntos saturados ({n_saturated/len(df)*100:.1f}%)',
                    transform=axes[0].transAxes, fontsize=10, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='red', alpha=0.3))
    
    axes[0].axhline(y=2.5, color='orange', linestyle='--', linewidth=2, label='Límite configurado (2.5 rad/s)')
    axes[0].axhline(y=3.5, color='r', linestyle='--', linewidth=2, label='PELIGROSO (>3.5 rad/s)')
    axes[0].set_ylabel('Velocidad Requerida (rad/s)', fontsize=11)
    axes[0].set_title('Velocidades Requeridas por Articulación', fontsize=12, fontweight='bold')
    axes[0].legend(loc='upper right', fontsize=8, ncol=2)
    axes[0].grid(True, alpha=0.3)
    
    # Subplot 2: Time from start (adaptativo)
    axes[1].plot(df['time'], df['time_from_start'] * 1000, 'b-', linewidth=2, label='time_from_start')
    axes[1].axhline(y=40, color='g', linestyle='--', linewidth=2, label='Mínimo (40ms)')
    axes[1].axhline(y=200, color='r', linestyle='--', linewidth=2, label='Máximo (200ms)')
    axes[1].set_ylabel('Tiempo dado al robot (ms)', fontsize=11)
    axes[1].set_title('Control Adaptativo: Tiempo para Alcanzar Objetivo', fontsize=12, fontweight='bold')
    axes[1].legend(loc='upper right', fontsize=10)
    axes[1].grid(True, alpha=0.3)
    
    # Subplot 3: Error vs Time from start (scatter)
    scatter = axes[2].scatter(df['error_norm'], df['time_from_start'] * 1000, 
                             c=df['vel_max_req'], cmap='RdYlGn_r', alpha=0.6, s=20)
    axes[2].set_xlabel('Error Articular Norm (rad)', fontsize=11)
    axes[2].set_ylabel('time_from_start (ms)', fontsize=11)
    axes[2].set_title('Correlación: Error vs Tiempo Adaptativo (color = vel_max_req)', fontsize=12, fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=axes[2])
    cbar.set_label('Vel Max Req (rad/s)', fontsize=10)
    
    fig.suptitle('Análisis de Velocidades y Control Adaptativo', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/velocity_analysis.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/velocity_analysis.png")
    plt.close()
    
    # ==== PLOT 2: Error de posición Cartesiano ====
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Subplot 1: Error articular por joint
    for i in range(6):
        axes[0].plot(df['time'], np.degrees(df[f'e_q{i}']), label=f'Joint {i+1}', linewidth=1.5, alpha=0.7)
    axes[0].set_ylabel('Error Articular (°)', fontsize=11)
    axes[0].set_title('Error de Seguimiento por Articulación', fontsize=12, fontweight='bold')
    axes[0].legend(loc='upper right', fontsize=9, ncol=3)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=0.5)
    
    # Subplot 2: Error norm total
    axes[1].plot(df['time'], np.degrees(df['error_q_norm']), 'r-', linewidth=2)
    axes[1].set_xlabel('Tiempo (s)', fontsize=11)
    axes[1].set_ylabel('Error Norm Total (°)', fontsize=11)
    axes[1].set_title('Error Articular Total', fontsize=12, fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    
    fig.suptitle('Análisis de Error de Seguimiento', fontsize=14, fontweight='bold')
    
    fig.suptitle('Análisis de Error de Seguimiento', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f"{PLOTS_DIR}/tracking_error.png", dpi=300, bbox_inches='tight')
    print(f"✓ Guardado: {PLOTS_DIR}/tracking_error.png")
    plt.close()
    
    # ==== PLOT 3: Joint positions - q_cmd vs q_actual ====
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
    
    # ==== PLOT 4: Trayectoria deseada 3D ====
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Trayectoria deseada
    ax.plot(df['pd0'], df['pd1'], df['pd2'], 
            'b-', linewidth=2, label='Trayectoria Deseada', alpha=0.8)
    
    # Punto inicial
    ax.scatter(df['pd0'].iloc[0], df['pd1'].iloc[0], df['pd2'].iloc[0], 
               c='green', s=150, marker='o', label='Inicio', edgecolors='k', linewidths=2)
    
    # Punto final
    ax.scatter(df['pd0'].iloc[-1], df['pd1'].iloc[-1], df['pd2'].iloc[-1], 
               c='red', s=150, marker='X', label='Final', edgecolors='k', linewidths=2)
    
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
