# aesa-processor
AESA 추적 레이더 데이터 처리부

## 인원 구성
| 이름 | 역할 |
|---|---|
| [신성현](https://github.com/sin6708k) | 프로젝트 매니저 |
| [박현준](https://github.com/2chasik) | 자세(`attitude`) 모듈과 빔(`beam`) 모듈 개발 |
| [이인성](https://github.com/inseong276-creator) | 통신(`comm`) 모듈 개발 |
| [장해찬](https://github.com/papperfield) | 오케스트레이션(`app`) 모듈 개발 |
| [천영기](https://github.com/YGC20) | 통신(`comm`) 모듈 개발 |

## 시스템 구성

```
레이더 데이터 처리부
├─ 자세(attitude) 모듈
├─ 빔(beam) 모듈
├─ 타겟(target) 모듈
├─ 통신(comm) 모듈
└─ 오케스트레이션(app) 모듈
```

- 외부 시스템과 통신 방식: TCP/IP
- 내부 모듈 간 통신 방식: 이벤트 큐