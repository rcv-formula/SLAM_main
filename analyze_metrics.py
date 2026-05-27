#!/usr/bin/env python3
"""
Cartographer 메트릭 분석 및 시각화
수집된 CSV/JSON 파일을 분석
"""

import pandas as pd
import json
from pathlib import Path
import argparse


def analyze_csv(csv_file):
    """CSV 파일 분석"""
    print(f"\n📊 CSV 분석: {csv_file}")
    print("="*80)
    
    df = pd.read_csv(csv_file)
    
    print(f"\n📈 기본 정보:")
    print(f"   데이터 포인트: {len(df)}")
    print(f"   메트릭: {len(df.columns) - 3}")
    print(f"   시간 범위: {df['elapsed_time'].min():.2f}s ~ {df['elapsed_time'].max():.2f}s")
    
    # 수치 메트릭만 필터링
    numeric_cols = df.select_dtypes(include=['float64', 'int64']).columns
    numeric_cols = [col for col in numeric_cols if col not in ['elapsed_time', 'iteration']]
    
    print(f"\n📊 메트릭별 통계:")
    print("-"*80)
    
    for col in sorted(numeric_cols):
        if col in df.columns:
            values = df[col].dropna()
            if len(values) > 0:
                print(f"\n{col}")
                print(f"   평균  : {values.mean():.6f}")
                print(f"   최소  : {values.min():.6f}")
                print(f"   최대  : {values.max():.6f}")
                print(f"   표준편차: {values.std():.6f}")


def plot_metrics(csv_file, metric_patterns=None):
    """메트릭 시각화"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("⚠️  matplotlib이 필요합니다: pip install matplotlib")
        return
    
    print(f"\n📈 그래프 생성 중...  {csv_file}")
    
    df = pd.read_csv(csv_file)
    
    # 수치 메트릭 필터링
    numeric_cols = df.select_dtypes(include=['float64', 'int64']).columns
    numeric_cols = [col for col in numeric_cols if col not in ['elapsed_time', 'iteration']]
    
    # 패턴 필터링
    if metric_patterns:
        filtered_cols = []
        for col in numeric_cols:
            for pattern in metric_patterns:
                if pattern in col:
                    filtered_cols.append(col)
                    break
        numeric_cols = filtered_cols
    
    if not numeric_cols:
        print("⚠️  그릴 메트릭이 없습니다")
        return
    
    # 그릴 메트릭 선택 (최대 6개)
    if len(numeric_cols) > 6:
        print(f"⚠️  메트릭이 많아서 처음 6개만 시각화합니다")
        numeric_cols = numeric_cols[:6]
    
    # 서브플롯 생성
    fig, axes = plt.subplots(len(numeric_cols), 1, figsize=(12, 3*len(numeric_cols)))
    
    if len(numeric_cols) == 1:
        axes = [axes]
    
    for ax, col in zip(axes, numeric_cols):
        values = df[col].dropna()
        ax.plot(df['elapsed_time'][:len(values)], values, label=col)
        ax.set_xlabel('Elapsed Time (s)')
        ax.set_ylabel('Value')
        ax.set_title(col)
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    plt.tight_layout()
    
    # 저장
    output_file = Path(csv_file).parent / f"{Path(csv_file).stem}_plot.png"
    plt.savefig(output_file, dpi=100)
    print(f"💾 그래프 저장: {output_file}")
    
    print("\n💡 팁: 다음 명령으로 실시간 모니터링할 수 있습니다:")
    print("   python monitor_metrics.py")


def compare_runs(csv_files):
    """여러 실행 비교"""
    print(f"\n📊 {len(csv_files)}개 실행 비교")
    print("="*80)
    
    dfs = []
    for csv_file in csv_files:
        df = pd.read_csv(csv_file)
        dfs.append((Path(csv_file).stem, df))
    
    # 공통 메트릭 찾기
    common_cols = set(dfs[0][1].columns)
    for _, df in dfs[1:]:
        common_cols &= set(df.columns)
    
    numeric_cols = [col for col in common_cols if col not in ['timestamp', 'elapsed_time', 'iteration']]
    
    print(f"\n📊 공통 메트릭: {len(numeric_cols)}")
    
    # 비교 테이블
    print("\n" + "-"*80)
    print(f"{'메트릭':<50} | {'평균':>15} | {'표준편차':>15}")
    print("-"*80)
    
    for col in sorted(numeric_cols)[:10]:  # 처음 10개만
        for name, df in dfs:
            values = df[col].dropna()
            if len(values) > 0:
                print(f"{name}: {col:<40} | {values.mean():>15.6f} | {values.std():>15.6f}")


def main():
    parser = argparse.ArgumentParser(description='메트릭 분석 도구')
    parser.add_argument('files', nargs='+', help='CSV 또는 JSON 파일')
    parser.add_argument('--plot', action='store_true', help='그래프 생성')
    parser.add_argument('--filter', help='메트릭 필터 (예: latency, residuals)')
    parser.add_argument('--compare', action='store_true', help='여러 파일 비교')
    
    args = parser.parse_args()
    
    if args.compare:
        compare_runs(args.files)
    else:
        for file in args.files:
            analyze_csv(file)
            
            if args.plot:
                patterns = args.filter.split(',') if args.filter else None
                plot_metrics(file, metric_patterns=patterns)


if __name__ == '__main__':
    main()
